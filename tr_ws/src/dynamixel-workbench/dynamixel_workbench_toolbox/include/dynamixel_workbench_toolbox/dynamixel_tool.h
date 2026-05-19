
#ifndef DYNAMIXEL_TOOL_H
#define DYNAMIXEL_TOOL_H

#include <string.h>
#include <stdio.h>

#include "dynamixel_item.h"

class DynamixelTool
{
 private:
  enum {DYNAMIXEL_BUFFER = 30};
  uint8_t dxl_id_[DYNAMIXEL_BUFFER];
  uint8_t dxl_cnt_;

  const char *model_name_;
  uint16_t model_number_;

  const ControlItem *control_table_;
  const ModelInfo *model_info_;

  uint16_t the_number_of_control_item_;

 public:
  DynamixelTool();
  ~DynamixelTool();

  void initTool(void);

  bool addTool(const char *model_name, uint8_t id, const char **log = NULL);
  bool addTool(uint16_t model_number, uint8_t id, const char **log = NULL);

  void addDXL(uint8_t id);

  const char *getModelName(void);
  uint16_t getModelNumber(void);

  const uint8_t* getID(void);
  uint8_t getDynamixelBuffer(void);
  uint8_t getDynamixelCount(void);

  float getRPM(void);

  int64_t getValueOfMinRadianPosition(void);
  int64_t getValueOfMaxRadianPosition(void);
  int64_t getValueOfZeroRadianPosition(void);

  float getMinRadian(void);
  float getMaxRadian(void);

  uint8_t getTheNumberOfControlItem(void);
  
  const ControlItem *getControlItem(const char *item_name, const char **log = NULL);
  const ControlItem *getControlTable(void);
  const ModelInfo *getModelInfo(void);

 private:
  bool setControlTable(const char *model_name, const char **log = NULL);
  bool setControlTable(uint16_t model_number, const char **log = NULL);

  bool setModelName(uint16_t model_number, const char **log = NULL);
  bool setModelNumber(const char *model_name, const char **log = NULL);
};
#endif //DYNAMIXEL_TOOL_H
