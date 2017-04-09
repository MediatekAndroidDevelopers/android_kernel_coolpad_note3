#ifndef _PERIPHERALS_COM_H_
#define _PERIPHERALS_COM_H_

int tp_device_id(int id);
void ctp_lock_mutex(void);
void ctp_unlock_mutex(void);

void Sensor_GetSensor_Index_info(int sensorindex, char * sensor_name_index,int *true_false);
 
#endif
