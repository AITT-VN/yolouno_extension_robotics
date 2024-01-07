Blockly.Blocks['i2c_dc_motor_driver'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "i2c_dc_motor_driver",
        "message0": "quay động cơ %1 tốc độ %2 (-100 đến 100)",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": [
              [
                "M1",
                "0"
              ],
              [
                "M2",
                "1"
              ],
              [
                "M3",
                "2"
              ],
              [
                "M4",
                "3"
              ]
            ]
          },
          {
            "type": "input_value",
            "name": "speed",
            "value": 0,
            "min": -100,
            "max": 100,
            "precision": 50
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": "#cb2026",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['i2c_dc_motor_driver'] = function (block) {
  Blockly.Python.definitions_['import_motor_driver'] = 'from i2c_motors_driver import *';
  Blockly.Python.definitions_['create_motor_driver'] = 'driver = DCMotor()\n';
  var dropdown_motor = block.getFieldValue('motor');
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'driver.setSpeed(' + dropdown_motor + ',' + speed + ')\n';
  return code;
};

Blockly.Blocks['i2c_dc_motor_delay'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "i2c_dc_motor_delay",
        "message0": "quay động cơ %1 với tốc độ %2 (0-100) trong %3 giây",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": [
              [
                "M1",
                "0"
              ],
              [
                "M2",
                "1"
              ],
              [
                "M3",
                "2"
              ],
              [
                "M4",
                "3"
              ]
            ]
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 50,
            name: "speed",
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            name: "time",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": "#cb2026",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["i2c_dc_motor_delay"] = function (block) {
  Blockly.Python.definitions_['import_motor_driver'] = 'from i2c_motors_driver import *';
  Blockly.Python.definitions_['create_motor_driver'] = 'driver = DCMotor()\n';
  var motor = block.getFieldValue('motor');
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var time = Blockly.Python.valueToCode(block, 'time', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'driver.set_motor_time(' + motor + ',' + speed + ", " + time + ')\n';
  return code;
};

Blockly.Blocks['i2c_dc_move_motor'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "i2c_dc_move_motor",
        "message0": "quay động cơ M1 tốc độ %1 M2 %2 M3 %3 M4 %4 (-100 đến 100)",
        "args0": [
          {
            "type": "input_value",
            "name": "motor1",
            "check": "Number",
          },
          {
            "type": "input_value",
            "name": "motor2",
            "check": "Number",
          },
          {
            "type": "input_value",
            "name": "motor3",
            "check": "Number",
          },
          {
            "type": "input_value",
            "name": "motor4",
            "check": "Number",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": "#cb2026",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["i2c_dc_move_motor"] = function (block) {
  Blockly.Python.definitions_['import_motor_driver'] = 'from i2c_motors_driver import *';
  Blockly.Python.definitions_['create_motor_driver'] = 'driver = DCMotor()\n';
  var motor1 = Blockly.Python.valueToCode(block, 'motor1', Blockly.Python.ORDER_ATOMIC);
  var motor2 = Blockly.Python.valueToCode(block, 'motor2', Blockly.Python.ORDER_ATOMIC);
  var motor3 = Blockly.Python.valueToCode(block, 'motor3', Blockly.Python.ORDER_ATOMIC);
  var motor4 = Blockly.Python.valueToCode(block, 'motor4', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'driver.setSpeed(0,' + motor1 + ')\n' + 'driver.setSpeed(1,' + motor2 + ')\n' + 'driver.setSpeed(2,' + motor3 + ')\n' + 'driver.setSpeed(3,' + motor4 + ')\n';
  return code;
};

Blockly.Blocks['i2c_stepper_motor_driver'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "i2c_dc_motor_driver",
        "message0": "động cơ bước %1 quay %3 tốc độ %2 (0 đến 250)",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "stepper",
            "options": [
              [
                "M1",
                "0"
              ],
              [
                "M2",
                "1"
              ]
            ]
          },
          {
            "type": "input_value",
            "name": "speed",
            "value": 0,
            "min": 0,
            "max": 250,
            "precision": 200
          },
          {
            "type": "field_dropdown",
            "name": "dir",
            "options": [
              [
                "tới",
                "0"
              ],
              [
                "lùi",
                "1"
              ]
            ]
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": "#cb2026",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['i2c_stepper_motor_driver'] = function (block) {
  Blockly.Python.definitions_['import_motor_driver'] = 'from i2c_motors_driver import *';
  Blockly.Python.definitions_['create_motor_driver'] = 'driver = StepperMotor()\n';
  var dropdown_motor = block.getFieldValue('stepper');
  var dropdown_dir = block.getFieldValue('dir');
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'driver.setSpeed(' + dropdown_motor + ',' + dropdown_dir + ',' + speed + ')\n';
  return code;
};

Blockly.Blocks['i2c_stepper_move_step'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "i2c_stepper_move_step",
        "message0": "động cơ bước %1 quay %3 %2 bước",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "stepper",
            "options": [
              [
                "M1",
                "0"
              ],
              [
                "M2",
                "1"
              ]
            ]
          },
          {
            "type": "input_value",
            "name": "step",
            "value": 0,
            "precision": 200
          },
          {
            "type": "field_dropdown",
            "name": "dir",
            "options": [
              [
                "tới",
                "0"
              ],
              [
                "lùi",
                "1"
              ]
            ]
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": "#cb2026",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['i2c_stepper_move_step'] = function (block) {
  Blockly.Python.definitions_['import_motor_driver'] = 'from i2c_motors_driver import *';
  Blockly.Python.definitions_['create_motor_driver'] = 'driver = StepperMotor()\n';
  var dropdown_motor = block.getFieldValue('stepper');
  var dropdown_dir = block.getFieldValue('dir');
  var step = Blockly.Python.valueToCode(block, 'step', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'driver.step(' + dropdown_motor + ',' + dropdown_dir + ',' + step + ')\n';
  return code;
};

Blockly.Blocks['i2c_stepper_release'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "i2c_stepper_move_step",
        "message0": "dừng động cơ bước %1 ",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "stepper",
            "options": [
              [
                "M1",
                "0"
              ],
              [
                "M2",
                "1"
              ]
            ]
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": "#cb2026",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['i2c_stepper_release'] = function (block) {
  Blockly.Python.definitions_['import_motor_driver'] = 'from i2c_motors_driver import *';
  Blockly.Python.definitions_['create_motor_driver'] = 'driver = StepperMotor()\n';
  var dropdown_motor = block.getFieldValue('stepper');
  // TODO: Assemble Python into code variable.
  var code = 'driver.release(' + dropdown_motor + ')\n';
  return code;
};
