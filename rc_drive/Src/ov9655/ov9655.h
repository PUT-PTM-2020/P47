/*
 * ov9655.h
 *  Created on: May 18, 2020
 */

#ifndef OV9655_OV9655_H_
#define OV9655_OV9655_H_

#define OV9655_MODE_QVGA_RGB565 0
#define OV9655_MODE_QVGA_YUV    1

#define OV9655_CAP_CONTINUOUS   0
#define OV9655_CAP_SINGLE_FRAME 1

RET ov9655_init(DCMI_HandleTypeDef *p_hdcmi, DMA_HandleTypeDef *p_hdma_dcmi, I2C_HandleTypeDef *p_hi2c);
RET ov9655_config(uint32_t mode);
RET ov9655_startCap(uint32_t capMode, uint32_t destAddress);
RET ov9655_stopCap();
void ov9655_registerCallback(void (*cbHsync)(uint32_t h), void (*cbVsync)(uint32_t v));
#endif /* OV9655_OV9655_H_ */
