#ifndef LK_LM75_H//[
#define LK_LM75_H

#define LM75_RET_SUCCESS			(0) // all right .
#define LM75_RET_PARAMERR			(-1) // parameter error !
#define LM75_RET_I2CCHN_NOTFOUND	(-2) // i2c adapter not found !
#define LM75_RET_NEWDEVFAIL			(-3) // register i2c client fail !!
#define LM75_RET_CHIP_NOTFOUND		(-4) // hardware chip not found !
#define LM75_RET_I2CTRANS_ERR		(-5) // i2c trans error !
#define LM75_RET_INITNOTYET			(-6) // should init first !


int lm75_release(void);
int lm75_init(int iPort);
int lm75_get_temperature(int iChipIdx,int *O_piTemperature);


#endif //]LK_LM75_H
