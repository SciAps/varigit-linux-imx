#ifndef OV5645_AF_H
#define OV5645_AF_H

int ov5645_af_init(struct i2c_client *sensor_i2c_client, bool verify);
int ov5645_af_constant_focus(struct i2c_client *sensor_i2c_client);
int ov5645_af_cancel_focus(struct i2c_client *sensor_i2c_client);
int ov5645_af_pause_focus(struct i2c_client *sensor_i2c_client);
bool ov5645_af_check_sensor_id(struct i2c_client *sensor_i2c_client, uint16_t sensor_id, bool init);

#endif
