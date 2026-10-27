/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * USB device library (ST's V2.x, lib/main/STM32F4) <-> PCD glue for the STM32H730's USB1_OTG_HS core with its full speed PHY
 * (PA11/PA12), as in src/main/vcp_hal/usbd_conf_stm32h7xx.c.
 */

#include "stm32h7xx_hal.h"

#include "usbd_core.h"
#include "usbd_dfu.h"

static PCD_HandleTypeDef hpcd;

void OTG_HS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

void *USBD_static_malloc(uint32_t size)
{
    static uint32_t memory[(sizeof(USBD_DFU_HandleTypeDef) + 3) / 4];
    return size <= sizeof(memory) ? memory : NULL;
}

void USBD_static_free(void *p)
{
    (void)p;
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
    (void)pcd;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    const GPIO_InitTypeDef init = {
        .Pin = GPIO_PIN_11 | GPIO_PIN_12,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF10_OTG1_HS,
    };
    HAL_GPIO_Init(GPIOA, (GPIO_InitTypeDef *)&init);

    __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

    HAL_NVIC_SetPriority(OTG_HS_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd)
{
    (void)pcd;

    __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_SetupStage(pcd->pData, (uint8_t *)pcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_DataOutStage(pcd->pData, epnum, pcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_DataInStage(pcd->pData, epnum, pcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_SOF(pcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_Reset(pcd->pData);
    USBD_LL_SetSpeed(pcd->pData, USBD_SPEED_FULL);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_Suspend(pcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_Resume(pcd->pData);
}

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete(pcd->pData, epnum);
}

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *pcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete(pcd->pData, epnum);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_DevConnected(pcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *pcd)
{
    USBD_LL_DevDisconnected(pcd->pData);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd.Instance = USB1_OTG_HS;
    hpcd.Init.dev_endpoints = 9;
    hpcd.Init.use_dedicated_ep1 = DISABLE;
    hpcd.Init.ep0_mps = DEP0CTL_MPS_64;
    hpcd.Init.low_power_enable = DISABLE;
    hpcd.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd.Init.Sof_enable = DISABLE;
    hpcd.Init.battery_charging_enable = DISABLE;
    hpcd.Init.speed = PCD_SPEED_FULL;
    hpcd.Init.vbus_sensing_enable = DISABLE;
    hpcd.Init.dma_enable = DISABLE;
    hpcd.Init.lpm_enable = DISABLE;

    hpcd.pData = pdev;
    pdev->pData = &hpcd;

    if (HAL_PCD_Init(&hpcd) != HAL_OK) {
        return USBD_FAIL;
    }

    // DFU uses the control endpoint only
    HAL_PCDEx_SetRxFiFo(&hpcd, 0x80);
    HAL_PCDEx_SetTxFiFo(&hpcd, 0, 0x40);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_DeInit(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Start(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    HAL_PCD_Stop(pdev->pData);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
    HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Close(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_Flush(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
    return USBD_OK;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *pcd = pdev->pData;

    if (ep_addr & 0x80) {
        return pcd->IN_ep[ep_addr & 0x7F].is_stall;
    }
    return pcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    HAL_PCD_SetAddress(pdev->pData, dev_addr);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
    HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size)
{
    HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
    return USBD_OK;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t delay)
{
    HAL_Delay(delay);
}
