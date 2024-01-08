var roboticsBlockColor = "#ff4ccd";


var robotics_stop_then = [
  [
    "dừng lại",
    "STOP_COAST"
  ],
  [
    "thắng gấp",
    "STOP_BRAKE"
  ],
  [
    "không làm gì",
    "None"
  ]
];

var robotics_motors = [
  [
    "motor1",
    "motor1"
  ],
  [
    "motor2",
    "motor2"
  ],
  [
    "motor3",
    "motor3"
  ],
  [
    "motor4",
    "motor4"
  ],
]

Blockly.Blocks['robotics_motor2p_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_init",
        "message0": "khởi tạo %1 IN1 %2 IN2 %3",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": robotics_motors,
          },
          {
            "type": "field_dropdown",
            "name": "in1",
            "options": digitalPins,
          },
          {
            "type": "field_dropdown",
            "name": "in2",
            "options": digitalPins,
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_motor2p_init"] = function (block) {
  var motor = block.getFieldValue("motor");
  var in1 = block.getFieldValue("in1");
  var in2 = block.getFieldValue("in2");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_motor'] = 'from robotics_motor import *';
  Blockly.Python.definitions_['init_motor_' + motor] = motor + ' = DCMotor2PIN(' + in1 + '_PIN, ' + in2 + '_IN)';
  var code = "";
  return code;
};

Blockly.Blocks['robotics_motor3p_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_init",
        "message0": "khởi tạo %1 IN1 %2 IN2 %3 PWM %4 STDBY %5",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": robotics_motors,
          },
          {
            "type": "field_dropdown",
            "name": "in1",
            "options": digitalPins,
          },
          {
            "type": "field_dropdown",
            "name": "in2",
            "options": digitalPins,
          },
          {
            "type": "field_dropdown",
            "name": "pwm",
            "options": digitalPins,
          },
          {
            "type": "field_dropdown",
            "name": "stdby",
            "options": digitalPins,
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_motor3p_init"] = function (block) {
  var motor = block.getFieldValue("motor");
  var in1 = block.getFieldValue("in1");
  var in2 = block.getFieldValue("in2");
  var pwm = block.getFieldValue("pwm");
  var stdby = block.getFieldValue("stdby");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_motor'] = 'from robotics_motor import *';
  if (stdby == 'None') {
    Blockly.Python.definitions_['init_motor_' + motor] = motor + 
      ' = DCMotor3PIN(' + in1 + '_PIN, ' + in2 + '_IN, ' +
      pwm + '_PIN, None)';
  } else {
    Blockly.Python.definitions_['init_motor_' + motor] = motor + 
      ' = DCMotor3PIN(' + in1 + '_PIN, ' + in2 + '_IN, ' +
      pwm + '_PIN, ' + stdby + '_PIN)';
  }
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_motori2c_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_init",
        "message0": "khởi tạo %1 với motor driver I2C V1 cổng %2",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": robotics_motors,
          },
          {
            "type": "field_dropdown",
            "name": "index",
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
            ],
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_motori2c_init"] = function (block) {
  var motor = block.getFieldValue("motor");
  var index = block.getFieldValue("index");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_motor'] = 'from robotics_motor import *';
  Blockly.Python.definitions_['init_motor_driver_v1'] = 'motor_driver_v1 = MotorDriver()';
  Blockly.Python.definitions_['init_motor_' + motor] = motor + ' = DCMotorI2CV1(motor_driver_v1, ' + index + ')';
  var code = "";
  return code;
};

Blockly.Blocks['robotics_motor_move'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_move",
        "message0": "động cơ %1 tốc độ %2",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": robotics_motors,
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 70,
            name: "speed",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};


Blockly.Python["robotics_motor_move"] = function (block) {
  var motor = block.getFieldValue("motor");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = motor + ".speed(" + speed + ")\n";

  return code;
};


Blockly.Blocks['robotics_robot2wd_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot2wd_init",
        "message0": "khởi tạo robot 2 động cơ trái %1 phải %2",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor1",
            "options": robotics_motors,
          },
          {
            "type": "field_dropdown",
            "name": "motor2",
            "options": robotics_motors,
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot2wd_init"] = function (block) {
  var motor1 = block.getFieldValue("motor1");
  var motor2 = block.getFieldValue("motor2");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_drivebase'] = 'from robotics_drivebase import *';
  Blockly.Python.definitions_['init_robot'] = 'robot = Robot2WD(' + motor1 + ', ' + motor2 + ')';
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_robot4wd_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot4wd_init",
        "message0": "khởi tạo robot 4 động cơ %1 trái - trước %2 phải - trước %3 %4 trái - sau %5 phải - sau %6",
        "args0": [
          {
            "type": "input_dummy"
          },
          {
            "type": "field_dropdown",
            "name": "motor1",
            "options": robotics_motors
          },
          {
            "type": "field_dropdown",
            "name": "motor2",
            "options": robotics_motors
          },
          {
            "type": "input_dummy"
          },
          {
            "type": "field_dropdown",
            "name": "motor3",
            "options": robotics_motors
          },
          {
            "type": "field_dropdown",
            "name": "motor4",
            "options": robotics_motors
          }
        ],
        "inputsInline": false,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot4wd_init"] = function (block) {
  var motor1 = block.getFieldValue("motor1");
  var motor2 = block.getFieldValue("motor2");
  var motor3 = block.getFieldValue("motor3");
  var motor4 = block.getFieldValue("motor4");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_drivebase'] = 'from robotics_drivebase import *';
  Blockly.Python.definitions_['init_robot'] = 'robot = Robot4WD(' + motor1 + ', ' + motor2 + ', ' + motor3 + ', ' + motor4 + ')';
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_robotmecanum_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robotmecanum_init",
        "message0": "khởi tạo robot Mecanum %1 trái - trước %2 phải - trước %3 %4 trái - sau %5 phải - sau %6",
        "args0": [
          {
            "type": "input_dummy"
          },
          {
            "type": "field_dropdown",
            "name": "motor1",
            "options": robotics_motors
          },
          {
            "type": "field_dropdown",
            "name": "motor2",
            "options": robotics_motors
          },
          {
            "type": "input_dummy"
          },
          {
            "type": "field_dropdown",
            "name": "motor3",
            "options": robotics_motors
          },
          {
            "type": "field_dropdown",
            "name": "motor4",
            "options": robotics_motors
          }
        ],
        "inputsInline": false,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robotmecanum_init"] = function (block) {
  var motor1 = block.getFieldValue("motor1");
  var motor2 = block.getFieldValue("motor2");
  var motor3 = block.getFieldValue("motor3");
  var motor4 = block.getFieldValue("motor4");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_drivebase'] = 'from robotics_drivebase import *';
  Blockly.Python.definitions_['init_robot'] = 'robot = RobotMecanum(' + motor1 + ', ' + motor2 + ', ' + motor3 + ', ' + motor4 + ')';
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_robot_move'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_move",
        "message0": "robot %1 tốc độ %2",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "direction",
            "options": [
              [
                {
                  "src": "static/blocks/block_images/59043.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "forward"
              ],
              [
                {
                  "src": "static/blocks/block_images/959159.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "backward"
              ],
              [
                {
                  "src": "static/blocks/block_images/860774.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "turn_left"
              ],
              [
                {
                  "src": "static/blocks/block_images/74474.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "turn_right"
              ]
            ]
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "speed",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_move"] = function (block) {
  var dir = block.getFieldValue("direction");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "await robot." + dir + "(" + speed + ")\n";
  return code;
};

Blockly.Blocks['robotics_robot_move_delay'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_move_delay",
        "message0": "robot %1 tốc độ %2 trong %3 giây",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "direction",
            "options": [
              [
                {
                  "src": "static/blocks/block_images/59043.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "forward"
              ],
              [
                {
                  "src": "static/blocks/block_images/959159.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "backward"
              ],
            ]
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "speed",
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 1,
            name: "time",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_move_delay"] = function (block) {
  var dir = block.getFieldValue("direction");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var time = Blockly.Python.valueToCode(block, 'time', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var  code = "await robot." + dir + "(" + speed + ", " + time + ")\n";

  return code;
};

Blockly.Blocks['robotics_robot_turn'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_turn",
        "message0": "robot xoay mức độ %1 tốc độ %2",
        "args0": [
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 50,
            name: "ratio",
          },
          {
            type: "input_value",
            check: "Number",
            value: 70,
            name: "speed",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_turn"] = function (block) {
  var ratio = Blockly.Python.valueToCode(block, 'ratio', Blockly.Python.ORDER_ATOMIC);
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "await robot.turn(" + ratio + ", " + speed + ")\n";

  return code;
};

Blockly.Blocks['robotics_robot_turn_delay'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_turn_delay",
        "message0": "robot xoay mức độ %1 tốc độ %2 trong %3 giây",
        "args0": [
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 50,
            name: "ratio",
          },
          {
            type: "input_value",
            check: "Number",
            value: 70,
            name: "speed",
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 1,
            name: "time",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_turn_delay"] = function (block) {
  var ratio = Blockly.Python.valueToCode(block, 'ratio', Blockly.Python.ORDER_ATOMIC);
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var time = Blockly.Python.valueToCode(block, 'time', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "await robot.turn(" + ratio + ", " + speed + ", " + time + ")\n";

  return code;
};


Blockly.Blocks['robotics_robot_stop'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_stop",
        "message0": "robot dừng di chuyển",
        "args0": [
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""

      }
    );
  }
};

Blockly.Python["robotics_robot_stop"] = function (block) {
  // TODO: Assemble Python into code variable.
  var code = "robot.stop()\n";
  return code;
};

// Line sensor and line following

Blockly.Blocks['robotics_line_sensorv2_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_line_sensorv2_init",
        "message0": "khởi tạo cảm biến dò line 4 mắt I2C V2",
        "args0": [],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_line_sensorv2_init"] = function (block) {
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_line_sensor'] = 'from robotics_line_sensor import *';
  Blockly.Python.definitions_['init_robotics_line_sensor'] = 'line_sensor = LineSensor_PCF8574()';
  var code = "robot.set_line_sensor(line_sensor)\n";
  return code;
};

Blockly.Blocks['robotics_line_sensor_digital_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_line_sensor_digital_init",
        "message0": "khởi tạo cảm biến dò line loại digital %1 S1 %2 S2 %3 S3 %4",
        "args0": [
          {
            "type": "input_dummy"
          },
          {
            "type": "field_dropdown",
            "name": "S1",
            "options": digitalPins
          },
          {
            "type": "field_dropdown",
            "name": "S2",
            "options": digitalPins
          },
          {
            "type": "field_dropdown",
            "name": "S3",
            "options": digitalPins
          },
        ],
        "inputsInline": false,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_line_sensor_digital_init"] = function (block) {
  var s1 = block.getFieldValue("S1");
  var s2 = block.getFieldValue("S2");
  var s3 = block.getFieldValue("S3");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_line_sensor'] = 'from robotics_line_sensor import *';
  if (s3 == 'None') {
    Blockly.Python.definitions_['init_robotics_line_sensor'] = 
      'line_sensor = LineSensor2P(' + s1 + '_PIN, ' + s2 + '_PIN)';  
  } else {
    Blockly.Python.definitions_['init_robotics_line_sensor'] = 
      'line_sensor = LineSensor3P(' + s1 + '_PIN, ' + s2 + '_PIN, ' + s3 + '_PIN)';  
  }
  var code = "robot.set_line_sensor(line_sensor)\n";
  return code;
  
};

Blockly.Blocks['robotics_follow_line_until_cross'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_until_cross",
        "message0": "dò line tốc độ %1 gặp vạch ngang rồi %2",
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 50,
            name: "speed",
          },
          {
            type: "field_dropdown",
            name: "stop",
            options: [
            ["dừng và khóa bánh", "STOP_BRAKE"],
            ["dừng lại", "STOP_COAST"],
            ["không làm gì", "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_until_cross"] = function (block) {
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_until_cross(" + speed + ", " + stop + ")\n";
  return code;
};

Blockly.Blocks['robotics_follow_line_until_end'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_until_end",
        "message0": "dò line tốc độ %1 đến cuối vạch đen rồi %2",
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 50,
            name: "speed",
          },
          {
            type: "field_dropdown",
            name: "stop",
            options: [
            ["dừng và khóa bánh", "STOP_BRAKE"],
            ["dừng lại", "STOP_COAST"],
            ["không làm gì", "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_until_end"] = function (block) {
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_until_end(" + speed + ", " + stop + ")\n";
  return code;
};

Blockly.Blocks['robotics_turn_until_line_detected_then'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_turn_until_line_detected_then",
        "message0": "quay %1 tốc độ %2 gặp vạch đen rồi %3",
        "args0": [
          {
            "type": "field_dropdown",
            "name": "direction",
            "options": [
              [
                {
                  "src": "static/blocks/block_images/860774.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "left"
              ],
              [
                {
                  "src": "static/blocks/block_images/74474.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "right"
              ]
            ]
          },
          {
            "type": "input_value",
            "name": "speed",
            "check": "Number",
          },
          {
            type: "field_dropdown",
            name: "stop",
            options: [
            ["dừng và khóa bánh", "STOP_BRAKE"],
            ["dừng lại", "STOP_COAST"],
            ["không làm gì", "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_turn_until_line_detected_then"] = function (block) {
  var dir = block.getFieldValue('direction');
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "";
  if (dir == "left") {
    code = "await robot.turn_until_line_detected(-100, " + speed + ", " + stop + ")\n";
  } else {
    code = "await robot.turn_until_line_detected(100, " + speed + ", " + stop + ")\n";
  }
  return code;
};

Blockly.Blocks['robotics_follow_line_until'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_until",
        "message0": "dò line tốc độ %1 đến khi %2",
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 50,
            name: "speed",
          },
          {
            "type": "input_value",
            "name": "condition",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_until"] = function (block) {
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var condition = Blockly.Python.valueToCode(block, 'condition', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_until(" + speed + ", " + "lambda: (" + condition + "))\n";
  return code;
};

