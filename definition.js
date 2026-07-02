var roboticsRobotBlockColor = "#ff4ccd";
var roboticsMotorBlockColor = "#0090f5";
var roboticsSensorBlockColor = "#9b6af6";
var roboticsLineBlockColor = "#34ccf1";

const ImgUrl2 = "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolobit_extension_rover/images/";

var robotics_stop_then = [
  [
    Blockly.Msg.ROBOTICS_STOP,
    "STOP"
  ],
  [
    Blockly.Msg.ROBOTICS_BRAKE,
    "BRAKE"
  ],
  [
    Blockly.Msg.ROBOTICS_NONE,
    "None"
  ]
];

var motor_stop_then = [
  [
    Blockly.Msg.ROBOTICS_STOP,
    "stop()"
  ],
  [
    Blockly.Msg.ROBOTICS_MOTOR_BRAKE,
    "brake()"
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
  [
    "motor5",
    "motor5"
  ],
  [
    "motor6",
    "motor6"
  ],
  [
    "motor7",
    "motor7"
  ],
  [
    "motor8",
    "motor8"
  ],
  [
    "motor9",
    "motor9"
  ],
  [
    "motor10",
    "motor10"
  ],
]

var robotics_motors_with_none = [
  [
    "______",
    "None"
  ],
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
  [
    "motor5",
    "motor5"
  ],
  [
    "motor6",
    "motor6"
  ],
  [
    "motor7",
    "motor7"
  ],
  [
    "motor8",
    "motor8"
  ],
  [
    "motor9",
    "motor9"
  ],
  [
    "motor10",
    "motor10"
  ],
  [
    "",
    "None"
  ],

]

var robotics_servos = [
  [
    "servo1",
    "servo1"
  ],
  [
    "servo2",
    "servo2"
  ],
  [
    "servo3",
    "servo3"
  ],
  [
    "servo4",
    "servo4"
  ],
  [
    "servo5",
    "servo5"
  ],
  [
    "servo6",
    "servo6"
  ],
  [
    "servo7",
    "servo7"
  ],
  [
    "servo8",
    "servo8"
  ]
]

Blockly.Blocks['robotics_motor2p_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_init",
        "message0": Blockly.Msg.ROBOTICS_MOTOR_INIT,
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
        "colour": roboticsMotorBlockColor,
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
  Blockly.Python.definitions_['import_motor'] = 'from motor import *';
  Blockly.Python.definitions_['init_motor_' + motor] = motor + ' = DCMotor2PIN(' + in1 + '_PIN, ' + in2 + '_IN)';
  var code = "";
  return code;
};

Blockly.Blocks['robotics_motor3p_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_init",
        "message0": Blockly.Msg.ROBOTICS_MOTOR_INIT1,
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
        "colour": roboticsMotorBlockColor,
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
  Blockly.Python.definitions_['import_robotics_motor'] = 'from motor import *';
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
        "type": "robotics_motori2c_init",
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_INIT1,
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
                "M1"
              ],
              [
                "M2",
                "M2"
              ],
              [
                "M3",
                "M3"
              ],
              [
                "M4",
                "M4"
              ],
              [
                "E1",
                "E1"
              ],
              [
                "E2",
                "E2"
              ]
            ],
          },          
          {
            "type": "field_dropdown",
            "name": "md",
            "options": [
              [
                "Control Hub",
                "3"
              ],
              [
                "Motor Driver V2",
                "2"
              ],
              [
                "Motor Driver V1",
                "1"
              ],
            ],
          },
          {
            "type": "field_checkbox",
            "name": "REVERSED",
            "checked": false
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_motori2c_init"] = function (block) {
  var motor = block.getFieldValue("motor");
  var index = block.getFieldValue("index");
  var md = block.getFieldValue("md");
  var reversed = block.getFieldValue('REVERSED') === 'TRUE';
  if (reversed) {
    reversed = 'True';
  } else {
    reversed = 'False';
  }
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_motor'] = 'from motor import *';
  if (md == 1) {
    Blockly.Python.definitions_['import_robotics_mdv1'] = 'from mdv1 import *';
    Blockly.Python.definitions_['init_motor_driver_v1'] = 'md_v1 = MotorDriverV1()';
    Blockly.Python.definitions_['init_motor_' + motor] = motor + ' = DCMotor(md_v1, ' + index + ', reversed=' + reversed + ')';
  } else {
    Blockly.Python.definitions_['import_robotics_mdv2'] = 'from mdv2 import *';
    Blockly.Python.definitions_['init_motor_driver_v2'] = 'md_v2 = MotorDriverV2()';
    Blockly.Python.definitions_['init_motor_' + motor] = motor + ' = DCMotor(md_v2, ' + index + ', reversed=' + reversed + ')';
  }
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_motor_run'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_run",
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_RUN,
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
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};


Blockly.Python["robotics_motor_run"] = function (block) {
  var motor = block.getFieldValue("motor");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = motor + ".run(" + speed + ")\n";

  return code;
};

//add motor brake

Blockly.Blocks['robotics_motor_brake'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_brake",
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_ACTION,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": robotics_motors,
          },
          {
            "type": "field_dropdown",
            "name": "action",
            "options": motor_stop_then,
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};


Blockly.Python["robotics_motor_brake"] = function (block) {
  var motor = block.getFieldValue("motor");
  var action = block.getFieldValue("action");
  // TODO: Assemble Python into code variable.
  var code = motor + "." + action + "\n";

  return code;
};

//
Blockly.Blocks['robotics_motor_set_encoder'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_set_encoder",
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_SET_ENCODER,
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
            name: "rpm",
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 70,
            name: "ppr",
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 70,
            name: "gears",
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_motor_set_encoder"] = function (block) {
  var motor = block.getFieldValue("motor");
  var rpm = Blockly.Python.valueToCode(block, 'rpm', Blockly.Python.ORDER_ATOMIC);
  var ppr = Blockly.Python.valueToCode(block, 'ppr', Blockly.Python.ORDER_ATOMIC);
  var gears = Blockly.Python.valueToCode(block, 'gears', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = motor + ".set_encoder(rpm=" + rpm + ", ppr=" + ppr + ", gears=" + gears + ")\n";

  return code;
};

Blockly.Blocks['robotics_motor_run_wait'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_run_wait",
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_RUN_WAIT,
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
            name: "amount",
          },
          {
            "type": "field_dropdown",
            "name": "unit",
            "options": [
              [
                Blockly.Msg.ROBOTICS_SECONDS,
                "second"
              ],
              [
                Blockly.Msg.ROBOTICS_ROUND,
                "rotation"
              ],
              [
                Blockly.Msg.ROBOTICS_DEGREE,
                "angle"
              ],
            ],
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
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};


Blockly.Python["robotics_motor_run_wait"] = function (block) {
  var motor = block.getFieldValue("motor");
  var amount = Blockly.Python.valueToCode(block, 'amount', Blockly.Python.ORDER_ATOMIC);
  var unit = block.getFieldValue("unit");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "";
  if (unit == "second") {
    code = "await " + motor + ".run_time(speed=" + speed + ", time=" + amount + "*1000, then=STOP)\n";
  } else if (unit == "angle") {
    code = "await " + motor + ".run_angle(speed=" + speed + ", angle=" + amount + ", then=BRAKE)\n";
  } else if (unit == "rotation") {
    code = "await " + motor + ".run_rotation(speed=" + speed + ", rotation=" + amount + ", then=BRAKE)\n";
  }

  return code;
};

Blockly.Blocks['robotics_motor_run_stalled'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_motor_run_stalled",
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_STALLED,
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
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};


Blockly.Python["robotics_motor_run_stalled"] = function (block) {
  var motor = block.getFieldValue("motor");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "await " + motor + ".run_until_stalled(" + speed + ", then=STOP)\n";

  return code;
};

Blockly.Blocks["robotics_motor_get"] = {
  init: function () {
    this.jsonInit({
      message0: "%1 %2",
      args0: [
        {
          type: "field_dropdown",
          name: "motor",
          options: robotics_motors,
        },
        {
          "type": "field_dropdown",
          "name": "property",
          "options": [
            [
              Blockly.Msg.ROBOTICS_GET_ANGLE,
              "angle()"
            ],
            [
              Blockly.Msg.ROBOTICS_GET_TICKS,
              "encoder_ticks()"
            ],
            [
              Blockly.Msg.ROBOTICS_GET_SPEED,
              "speed()"
            ]
          ],
        },
      ],
      output: null,
      colour: roboticsMotorBlockColor,
      tooltip: "",
      helpUrl: ""
    });
  }
};

Blockly.Python["robotics_motor_get"] = function (block) {
  var motor = block.getFieldValue('motor');
  var property = block.getFieldValue('property');
  // TODO: Assemble Python into code variable.
  var code = motor + '.' + property;
  // TODO: Change ORDER_NONE to the correct strength.
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks['robotics_motor_reset_angle'] = {
  /**
   * Block for waiting.
   * @this Blockly.Block
   */
  init: function () {
    this.jsonInit(
      {
        "message0": Blockly.Msg.ROBOTICS_I2C_MOTOR_RESET_TICKS,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "motor",
            "options": robotics_motors
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['robotics_motor_reset_angle'] = function (block) {
  var motor = block.getFieldValue('motor');
  // TODO: Assemble Python into code variable.
  var code = motor + ".reset_angle()\n";
  return code;
};

Blockly.Blocks['robotics_servo_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_servo_init",
        "message0": Blockly.Msg.ROBOTICS_SERVO_INIT,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "servo",
            "options": robotics_servos,
          },
          {
            "type": "field_dropdown",
            "name": "port",
            "options": [
              [
                "S1",
                "S1"
              ],
              [
                "S2",
                "S2"
              ],
              [
                "S3",
                "S3"
              ],
              [
                "S4",
                "S4"
              ],
              [
                "D2",
                "D2"
              ],
              [
                "D3",
                "D3"
              ],
              [
                "D4",
                "D4"
              ],
              [
                "D5",
                "D5"
              ],
              [
                "D6",
                "D6"
              ],
              [
                "D7",
                "D7"
              ],
              [
                "D8",
                "D8"
              ],
              [
                "D9",
                "D9"
              ],
              [
                "D10",
                "D10"
              ],
              [
                "D11",
                "D11"
              ],
              [
                "D12",
                "D12"
              ],
              [
                "D13",
                "D13"
              ],
              [
                "D0",
                "D0"
              ],
              [
                "D1",
                "D1"
              ],
            ],
          },          
          {
            "type": "field_dropdown",
            "name": "type",
            "options": [
              [
                "180",
                "180"
              ],
              [
                "270",
                "270"
              ],
              [
                "360",
                "360"
              ],
            ],
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_servo_init"] = function (block) {
  var servo = block.getFieldValue("servo");
  var port = block.getFieldValue("port");
  var type = block.getFieldValue("type");
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_servo'] = 'from servo import *';
  if (port.startsWith("S")) {
    Blockly.Python.definitions_['import_robotics_mdv2'] = 'from mdv2 import *';
    Blockly.Python.definitions_['init_motor_driver_v2'] = 'md_v2 = MotorDriverV2()';
    Blockly.Python.definitions_['init_robotics_servo_' + servo] = servo + ' = Servo(md_v2, ' + port + ', ' + type + ')';
  } else {
    Blockly.Python.definitions_['init_robotics_servo_' + servo] = servo + ' = Servo(' + port + '_PIN, ' + type + ')';
  }
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_servo_limit'] = {
  /**
   * Block for waiting.
   * @this Blockly.Block
   */
  init: function () {
    this.jsonInit(
      {
        "message0": Blockly.Msg.ROBOTICS_SERVO_LIMIT,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "servo",
            "options": robotics_servos,
          },
          {
            "type": "input_value",
            "name": "min",
            "check": "Number",
            "min": 0,
            "max": 270,
          },
          {
            "type": "input_value",
            "name": "max",
            "check": "Number",
            "min": 0,
            "max": 270,
          },
          {
            type: "input_dummy"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['robotics_servo_limit'] = function (block) {
  var servo = block.getFieldValue("servo");
  var min = Blockly.Python.valueToCode(block, 'min', Blockly.Python.ORDER_ATOMIC);
  var max = Blockly.Python.valueToCode(block, 'max', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = servo + '.limit(min=' + min + ', max=' + max + ')\n';
  return code;
};


Blockly.Blocks['robotics_servo_angle'] = {
  /**
   * Block for waiting.
   * @this Blockly.Block
   */
  init: function () {
    this.jsonInit(
      {
        "message0": Blockly.Msg.ROBOTICS_SERVO_ANGLE,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "servo",
            "options": robotics_servos,
          },
          {
            "type": "input_value",
            "name": "angle",
            "check": "Number",
            "min": 0,
            "max": 270,
          },
          {
            "type": "input_value",
            "name": "speed",
            "check": "Number",
            "min": 0,
            "max": 100,
          },
          {
            type: "input_dummy"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['robotics_servo_angle'] = function (block) {
  var servo = block.getFieldValue("servo");
  var angle = Blockly.Python.valueToCode(block, 'angle', Blockly.Python.ORDER_ATOMIC);
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'await ' + servo + '.run_angle(angle=' + angle + ', speed=' + speed + ')\n';
  return code;
};

Blockly.Blocks['robotics_servo_steps'] = {
  /**
   * Block for waiting.
   * @this Blockly.Block
   */
  init: function () {
    this.jsonInit(
      {
        "message0": Blockly.Msg.ROBOTICS_SERVO_STEP,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "servo",
            "options": robotics_servos,
          },
          {
            "type": "input_value",
            "name": "steps",
            "check": "Number",
            "min": 0,
            "max": 270,
          },
          {
            type: "input_dummy"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['robotics_servo_steps'] = function (block) {
  var servo = block.getFieldValue("servo");
  var steps = Blockly.Python.valueToCode(block, 'steps', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'await ' + servo + '.run_steps(' + steps + ')\n';
  return code;
};

Blockly.Blocks['robotics_servo_spin'] = {
  /**
   * Block for waiting.
   * @this Blockly.Block
   */
  init: function () {
    this.jsonInit(
      {
        "message0": Blockly.Msg.ROBOTICS_SERVO_SPIN,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "servo",
            "options": robotics_servos,
          },
          {
            "type": "input_value",
            "name": "speed",
            "check": "Number",
            "min": -100,
            "max": 100,
          },
          {
            type: "input_dummy"
          }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsMotorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python['robotics_servo_spin'] = function (block) {
  var servo = block.getFieldValue("servo");
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = servo + '.spin(' + speed + ')\n';
  return code;
};


Blockly.Blocks['robotics_robot_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_init",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_INIT,
        "args0": [
          {
            "type": "field_image",
            "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/left_front_wheel.svg",
            "width": 20,
            "height": 20,
            "alt": Blockly.Msg.ROBOTICS_FRONT_LEFT,
            "flipRtl": false
          },
          {
            "type": "field_dropdown",
            "name": "m1",
            "options": robotics_motors_with_none
          },
          {
            "type": "field_image",
            "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/right_front_wheel.svg",
            "width": 20,
            "height": 20,
            "alt": Blockly.Msg.ROBOTICS_FRONT_RIGHT,
            "flipRtl": false
          },
          {
            "type": "field_dropdown",
            "name": "m2",
            "options": robotics_motors_with_none
          },
          {
            "type": "field_image",
            "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/left_rear_wheel.svg",
            "width": 20,
            "height": 20,
            "alt": Blockly.Msg.ROBOTICS_BACK_LEFT,
            "flipRtl": false
          },
          {
            "type": "field_dropdown",
            "name": "m3",
            "options": robotics_motors_with_none
          },
          {
            "type": "field_image",
            "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/right_rear_wheel.svg",
            "width": 20,
            "height": 20,
            "alt": Blockly.Msg.ROBOTICS_BACK_RIGHT,
            "flipRtl": false
          },
          {
            "type": "field_dropdown",
            "name": "m4",
            "options": robotics_motors_with_none
          },
          {
            "type": "field_checkbox",
            "name": "mecanum",
            "checked": false
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_init"] = function (block) {
  var motor1 = block.getFieldValue("m1");
  var motor2 = block.getFieldValue("m2");
  var motor3 = block.getFieldValue("m3");
  var motor4 = block.getFieldValue("m4");
  var mecanum = block.getFieldValue('mecanum') === 'TRUE';
  
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_drivebase'] = 'from drivebase import *';
  var mode = 'MODE_2WD';
  if (mecanum) {
    mode = 'MODE_MECANUM';
  } else {
    if (motor1 != "None" && motor2 != "None" && motor3 != "None" && motor4 != "None") {
      mode = 'MODE_4WD';
    } else {
      mode = 'MODE_2WD';
    }
  }

  Blockly.Python.definitions_['init_robotics_drivebase'] = 'robot = DriveBase(' + mode + ', m1=' + motor1 + ', m2=' + motor2 + ', m3=' + motor3 + ', m4=' + motor4 + ')';
  Blockly.Python.definitions_['deinit_robot'] = 'robot.stop()';
  
  var code = "";
  return code;
};

Blockly.Blocks['robotics_robot_config'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_config",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_CONFIG,
        "args0": [
          {
            "type": "field_image",
            "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/wheel_diameter.svg",
            "width": 30,
            "height": 30,
            "alt": Blockly.Msg.ROBOTICS_WHEELS,
            "flipRtl": false
          },
          {
            "type": "input_value",
            "name": "wheel",
            "check": "Number",
            "min": 0,
          },
          {
            "type": "field_image",
            "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/robot_width.svg",
            "width": 30,
            "height": 30,
            "alt": Blockly.Msg.ROBOTICS_WIDTH,
            "flipRtl": false
          },
          {
            "type": "input_value",
            "name": "width",
            "check": "Number",
            "min": 0,
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_config"] = function (block) {
  var wheel = Blockly.Python.valueToCode(block, 'wheel', Blockly.Python.ORDER_ATOMIC);
  var width = Blockly.Python.valueToCode(block, 'width', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.  
  var code = "robot.size(wheel=" + wheel + ", width=" + width + ")\n";
  return code;
};

Blockly.Blocks['robotics_robot_move'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_move",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_MOVE,
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
              ],
              [
                {
                  "src": "static/blocks/block_images/arrow-left.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "side left"
                },
                "move_left"
              ],
              [
                {
                  "src": "static/blocks/block_images/arrow-right.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "side right"
                },
                "move_right"
              ]
            ]
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
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
  var code = "robot." + dir + "()\n";
  return code;
};

Blockly.Blocks['robotics_robot_move_delay'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_move_delay",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_MOVE_DELAY,
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
                "forward_for"
              ],
              [
                {
                  "src": "static/blocks/block_images/959159.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "backward_for"
              ],
              [
                {
                  "src": "static/blocks/block_images/arrow-left.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "side left"
                },
                "move_left_for"
              ],
              [
                {
                  "src": "static/blocks/block_images/arrow-right.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "side right"
                },
                "move_right_for"
              ]
            ]
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "amount",
          },
          {
            "type": "field_dropdown",
            "name": "unit",
            "options": [
              [
                Blockly.Msg.ROBOTICS_SECONDS,
                "SECOND"
              ],
              [
                "cm",
                "CM"
              ],
            ],
          },
          {
            "type": "field_dropdown",
            "name": "then",
            "options": [
              [
                Blockly.Msg.ROBOTICS_STOP,
                "STOP"
              ],
              [
                Blockly.Msg.ROBOTICS_BRAKE,
                "BRAKE"
              ],
            ],
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_move_delay"] = function (block) {
  var dir = block.getFieldValue("direction");
  var unit = block.getFieldValue("unit");
  var then = block.getFieldValue("then");
  var amount = Blockly.Python.valueToCode(block, 'amount', Blockly.Python.ORDER_ATOMIC);

  var code = "await robot." + dir + "(" + amount + ", unit=" + unit + ", then=" + then + ")\n";

  return code;
};

Blockly.Blocks['robotics_robot_turn_delay'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_turn",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_TURN_DELAYED,
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
                "turn_left_for"
              ],
              [
                {
                  "src": "static/blocks/block_images/74474.svg",
                  "width": 15,
                  "height": 15,
                  "alt": "*"
                },
                "turn_right_for"
              ]
            ]
          },
          {
            min: 0,
            type: "input_value",
            check: "Number",
            value: 1,
            name: "amount",
          },
          {
            "type": "field_dropdown",
            "name": "unit",
            "options": [
              [
                Blockly.Msg.ROBOTICS_SECONDS,
                "SECOND"
              ],
              [
                Blockly.Msg.ROBOTICS_DEGREE,
                "DEGREE"
              ],
            ],
          },
          {
            "type": "field_dropdown",
            "name": "then",
            "options": [
              [
                Blockly.Msg.ROBOTICS_STOP,
                "STOP"
              ],
              [
                Blockly.Msg.ROBOTICS_BRAKE,
                "BRAKE"
              ],
            ],
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_turn_delay"] = function (block) {
  var dir = block.getFieldValue("direction");
  var unit = block.getFieldValue("unit");
  var amount = Blockly.Python.valueToCode(block, 'amount', Blockly.Python.ORDER_ATOMIC);
  var then = block.getFieldValue("then");
  var code = "await robot." + dir + "(" + amount + ", unit=" + unit + ", then=" + then + ")\n";
  return code;
};

Blockly.Blocks['robotics_robot_stop'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_stop",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_STOP,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "then",
            "options": [
              [
                Blockly.Msg.ROBOTICS_STOP,
                "stop"
              ],
              [
                Blockly.Msg.ROBOTICS_BRAKE,
                "brake"
              ],
            ],
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""

      }
    );
  }
};

Blockly.Python["robotics_robot_stop"] = function (block) {
  // TODO: Assemble Python into code variable.
  var then = block.getFieldValue("then");
  var code = "robot." + then + "()\n";
  return code;
};

Blockly.Blocks['robotics_robot_set_speed'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_set_speed",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_SET_SPEED,
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "speed",
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "min_speed",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_set_speed"] = function (block) {
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC);
  var min_speed = Blockly.Python.valueToCode(block, 'min_speed', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "robot.speed(" + speed + ", min_speed=" + min_speed + ")\n";
  return code;
};

Blockly.Blocks['robotics_robot_use_gyro'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_use_gyro",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_USE_GYRO,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "use_gyro",
            "options": [
              [
                "encoder",
                "False"
              ],
              [
                Blockly.Msg.ROBOTICS_ANGLE_SENSOR,
                "True"
              ],
            ],
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""

      }
    );
  }
};

Blockly.Python["robotics_robot_use_gyro"] = function (block) {
  // TODO: Assemble Python into code variable.
  var use_gyro = block.getFieldValue("use_gyro");
  var code = "robot.use_gyro(" + use_gyro + ")\n";
  return code;
};

Blockly.Blocks['robotics_robot_set_pid'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_set_pid",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_SET_PID,
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "KP",
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "KI",
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "KD",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_set_pid"] = function (block) {
  var kp = Blockly.Python.valueToCode(block, 'KP', Blockly.Python.ORDER_ATOMIC);
  var ki = Blockly.Python.valueToCode(block, 'KI', Blockly.Python.ORDER_ATOMIC);
  var kd = Blockly.Python.valueToCode(block, 'KD', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "robot.pid(Kp=" + kp + ", Ki=" + ki + ", Kd=" + kd + ")\n";
  return code;
};

Blockly.Blocks['robotics_robot_set_speed_ratio'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_robot_set_speed_ratio",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_SET_SPEED_RATIO,
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "left",
          },
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "right",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsRobotBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_robot_set_speed_ratio"] = function (block) {
  var left = Blockly.Python.valueToCode(block, 'left', Blockly.Python.ORDER_ATOMIC);
  var right = Blockly.Python.valueToCode(block, 'right', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = "robot.speed_ratio(left=" + left + ", right=" + right + ")\n";
  return code;
};

// REMOTE CONTROL BLOCK

const ImgUrl = 'https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/xbot_extension_robocon/images/';

Blockly.Blocks['robotics_remote_control_init'] = {
  init: function () {
    this.jsonInit(
      {
        type: "robotics_remote_control_init",
        message0: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_INIT,
        previousStatement: null,
        nextStatement: null,
        args0: [ 
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "accel_steps",
          },
        ],
        colour: roboticsRobotBlockColor,
        "inputsInline": true,
        tooltip: "",
        helpUrl: ""
      }
    )
  },
};


Blockly.Python['robotics_remote_control_init'] = function (block) {
  var steps = Blockly.Python.valueToCode(block, 'accel_steps', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_ble'] = 'from ble import *';
  Blockly.Python.definitions_['import_robotics_gamepad'] = 'from gamepad import *';
  Blockly.Python.definitions_['init_robotics_gamepad'] = 'gamepad = Gamepad()';
  
  var code = 'create_task(ble.wait_for_msg())\n';
  code += 'create_task(gamepad.run())\n';
  code += 'create_task(robot.run_teleop(gamepad, accel_steps=' + steps + '))\n';
  return code;
};


Blockly.Blocks['robotics_remote_control_side_move_mode'] = {
  init: function () {
    this.jsonInit(
      {
        type: "robotics_remote_control_side_move_mode",
        message0: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_SIDE_MOVE_MODE,
        previousStatement: null,
        nextStatement: null,
        args0: [
          {
            "type": "field_dropdown",
            "name": "action",
            "options": [
              [
                Blockly.Msg.ROBOTICS_DPAD,
                "DPAD"
              ],
              [
                Blockly.Msg.ROBOTICS_LEFT_JOYSTICK,
                "JOYSTICK"
              ],
            ],
          },
         ],
        colour: roboticsRobotBlockColor,
        "inputsInline": true,
        tooltip: "",
        helpUrl: ""
      }
    )
  },
};


Blockly.Python['robotics_remote_control_side_move_mode'] = function (block) {
  // TODO: Assemble Python into code variable.
  var action = block.getFieldValue("action");
  var code = "robot.side_move_mode = " + action + "\n";
  return code;
};

Blockly.Blocks['robotics_remote_control_off'] = {
  init: function () {
    this.jsonInit(
      {
        type: "robotics_remote_control_off",
        message0: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_OFF,
        previousStatement: null,
        nextStatement: null,
        args0: [
          {
            "type": "field_dropdown",
            "name": "action",
            "options": [
              [
                Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_PAUSE,
                "True"
              ],
              [
                Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_RESTART,
                "False"
              ],
            ],
          },
         ],
        colour: roboticsRobotBlockColor,
        "inputsInline": true,
        tooltip: "",
        helpUrl: ""
      }
    )
  },
};


Blockly.Python['robotics_remote_control_off'] = function (block) {
  // TODO: Assemble Python into code variable.
  var action = block.getFieldValue("action");
  var code = "robot.mode_auto = " + action + "\n";
  return code;
};

Blockly.Blocks["robotics_remote_control_on_button"] = {
  init: function () {
    this.jsonInit({
      colour: roboticsRobotBlockColor,
      message0: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_ON_BUTTON,
      tooltip: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_ON_BUTTON_TOOLTIP,
      args0: [
        {
          type: "field_dropdown",
          name: "BUTTON",
          options: [
            [
              {
                "src": "static/blocks/block_images/59043.svg",
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_UP"
            ],
            [
              {
                "src": "static/blocks/block_images/959159.svg",
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_DOWN"
            ],
            [
              {
                "src": "static/blocks/block_images/arrow-left.svg",
                "width": 15,
                "height": 15,
                "alt": "side left"
              },
              "BTN_LEFT"
            ],
            [
              {
                "src": "static/blocks/block_images/arrow-right.svg",
                "width": 15,
                "height": 15,
                "alt": "side right"
              },
              "BTN_RIGHT"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-square.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_SQUARE"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-circle.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_CIRCLE"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-cross.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_CROSS"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-triangle.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_TRIANGLE"
            ],
            ["L1", "BTN_L1"],
            ["R1", "BTN_R1"],
            ["L2", "BTN_L2"],
            ["R2", "BTN_R2"],
            ["SHARE", "BTN_M1"],
            ["OPTIONS", "BTN_M2"],
            ["Left Joystick", "BTN_THUMBL"],
            ["Right Joystick", "BTN_THUMBR"],
          ],
        },
        {
          type: "input_dummy",
        },
        {
          type: "input_statement",
          name: "ACTION",
        },
      ],
      helpUrl: "",
    });
  }
};

Blockly.Python['robotics_remote_control_on_button'] = function (block) {
  var button = block.getFieldValue('BUTTON');
  var statements_action = Blockly.Python.statementToCode(block, 'ACTION');

  var globals = buildGlobalString(block);

  var cbFunctionName = Blockly.Python.provideFunction_(
    'on_cmd_' + button,
    (globals != '') ?
      ['def ' + Blockly.Python.FUNCTION_NAME_PLACEHOLDER_ + '():',
        globals,
        statements_action || Blockly.Python.PASS
      ] :
      ['def ' + Blockly.Python.FUNCTION_NAME_PLACEHOLDER_ + '():',
        statements_action || Blockly.Python.PASS
      ]);

  var code = 'robot.on_teleop_command(' + button + ', ' + cbFunctionName + ')';
  Blockly.Python.definitions_['setup_robotics_on_teleop_command' + button] = code;

  return '';
};

Blockly.Blocks["robotics_remote_control_read_button"] = {
  init: function () {
    this.jsonInit({
      colour: roboticsRobotBlockColor,
      tooltip: "",
      message0: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_READ_BUTTON,
      args0: [
        {
          type: "field_dropdown",
          name: "BUTTON",
          options: [
            [
              {
                "src": "static/blocks/block_images/59043.svg",
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_UP"
            ],
            [
              {
                "src": "static/blocks/block_images/959159.svg",
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_DOWN"
            ],
            [
              {
                "src": "static/blocks/block_images/arrow-left.svg",
                "width": 15,
                "height": 15,
                "alt": "side left"
              },
              "BTN_LEFT"
            ],
            [
              {
                "src": "static/blocks/block_images/arrow-right.svg",
                "width": 15,
                "height": 15,
                "alt": "side right"
              },
              "BTN_RIGHT"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-square.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_SQUARE"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-circle.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_CIRCLE"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-cross.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_CROSS"
            ],
            [
              {
                "src": 'static/blocks/block_images/gamepad-triangle.png',
                "width": 15,
                "height": 15,
                "alt": "*"
              },
              "BTN_TRIANGLE"
            ],
            ["L1", "BTN_L1"],
            ["R1", "BTN_R1"],
            ["L2", "BTN_L2"],
            ["R2", "BTN_R2"],
          ],
        }
      ],
      output: "Boolean",
      helpUrl: "",
    });
  },
};

Blockly.Python["robotics_remote_control_read_button"] = function (block) {
  var button = block.getFieldValue("BUTTON");
  // TODO: Assemble Python into code variable.
  var code = 'gamepad.data[' + button + '] == 1';
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks["robotics_remote_control_read_joystick"] = {
  init: function () {
    this.jsonInit({
      colour: roboticsRobotBlockColor,
      tooltip: "",
      message0: Blockly.Msg.ROBOTICS_ROBOT_REMOTE_CONTROL_READ_JOYSTICK,
      args0: [
        {
          "type": "field_dropdown",
          "name": "joystick",
          "options": [
            [
              Blockly.Msg.ROBOTICS_LEFT,
              "AL"
            ],
            [
              Blockly.Msg.ROBOTICS_RIGHT,
              "AR"
            ]
          ]
        },
        {
          "type": "field_dropdown",
          "name": "data",
          "options": [
            [
              "X",
              "X"
            ],
            [
              "Y",
              "Y"
            ],
            [
              Blockly.Msg.ROBOTICS_DIR,
              "_DIR"
            ],
            [
              Blockly.Msg.ROBOTICS_DISTANCE,
              "_DISTANCE"
            ]
          ]
        }
      ],
      output: "Number",
      helpUrl: "",
    });
  },
};

Blockly.Python["robotics_remote_control_read_joystick"] = function (block) {
  var joystick = block.getFieldValue("joystick");
  var data = block.getFieldValue("data");
  // TODO: Assemble Python into code variable.
  var code = 'gamepad.data[' + joystick + data + ']';
  return [code, Blockly.Python.ORDER_NONE];
};


// Angle sensor


Blockly.Blocks['robotics_angle_sensor_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_angle_sensor_init",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_ANGLE_SENSOR_INIT,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "type",
            "options": [
              ["MPU6050", "MPU6050"],
              ["MPU9250", "MPU9250"]
            ]
          },
          {
            type: "input_value",
            check: "Number",
            value: 100,
            name: "samples",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsSensorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_angle_sensor_init"] = function (block) {
  var type = block.getFieldValue("type");
  var samples = Blockly.Python.valueToCode(block, 'samples', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  if (type == "MPU6050") {
    Blockly.Python.definitions_['import_robotics_mpu6050'] = 'from mpu6050 import MPU6050';
    Blockly.Python.definitions_['init_robotics_mpu6050'] = 'imu = MPU6050()';
  }

  if (type == "MPU9250") {
    Blockly.Python.definitions_['import_robotics_mpu9250'] = 'from robotics_mpu9250 import MPU9250';  
    Blockly.Python.definitions_['init_robotics_mpu9250'] = 'imu = MPU9250()';
  }
  
  Blockly.Python.definitions_['import_robotics_angle_sensor'] = 'from angle_sensor import AngleSensor';
  Blockly.Python.definitions_['init_robotics_angle_sensor'] = 'angle_sensor = AngleSensor(imu)';

  var code = 'angle_sensor.calibrate(' + samples + ')\n' + 
    'create_task(angle_sensor.run())\n' +
    'robot.angle_sensor(angle_sensor)\n';
    
  return code;
};

Blockly.Blocks['robotics_angle_sensor_calib'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_angle_sensor_calib",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_ANGLE_SENSOR_CALIB,
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 100,
            name: "samples",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsSensorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_angle_sensor_calib"] = function (block) {
  var samples = Blockly.Python.valueToCode(block, 'samples', Blockly.Python.ORDER_ATOMIC);
  // TODO: Assemble Python into code variable.
  var code = 'angle_sensor.calibrate(' + samples + ')\n' + 'await angle_sensor.reset()\n';
    
  return code;
};

Blockly.Blocks["robotics_angle_sensor_get"] = {
  init: function () {
    this.jsonInit({
      colour: roboticsSensorBlockColor,
      tooltip: "",
      message0: Blockly.Msg.ROBOTICS_ROBOT_READ_ANGLE_SENSOR,
      args0: [
        {
          type: "field_dropdown",
          name: "AXIS",
          options: [
            ["heading (yaw)", "heading"],
            ["pitch", "pitch"],
            ["roll", "roll"],
            [Blockly.Msg.ROBOTICS_ROBOT_READ_ALL_DATA, "print_data()"],
          ],
        }
      ],
      output: "Number",
      helpUrl: ""
    });
  },
};

Blockly.Python["robotics_angle_sensor_get"] = function (block) {
  var axis = block.getFieldValue("AXIS");
  // TODO: Assemble Python into code variable.
  var code = "angle_sensor." + axis;
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks["robotics_angle_sensor_get_imu"] = {
  init: function () {
    this.jsonInit({
      colour: roboticsSensorBlockColor,
      tooltip: "",
      message0: Blockly.Msg.ROBOTICS_ROBOT_GET_IMU,
      args0: [
        {
          type: "field_dropdown",
          name: "SENSOR",
          options: [
            ["accelerometer", "accel"],
            ["gyroscope", "gyro"],
            ["magnetometer", "mag"],
          ],
        },
        {
          type: "field_dropdown",
          name: "AXIS",
          options: [
            ["x", "x"],
            ["y", "y"],
            ["z", "z"],
          ],
        }
      ],
      output: "Number",
      helpUrl: ""
    });
  },
};

Blockly.Python["robotics_angle_sensor_get_imu"] = function (block) {
  var sensor = block.getFieldValue("SENSOR");
  var axis = block.getFieldValue("AXIS");
  // TODO: Assemble Python into code variable.
  var code = "imu." + sensor + "." + axis + "";
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks['robotics_angle_sensor_reset'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_angle_sensor_reset",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_ANGLE_RESET,
        "args0": [],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsSensorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_angle_sensor_reset"] = function (block) {
  // TODO: Assemble Python into code variable.
  var code = 'await angle_sensor.reset()\n';

  return code;
};

Blockly.Blocks['robotics_angle_sensor_config'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_angle_sensor_config",
        "message0": Blockly.Msg.ROBOTICS_ANGLE_SENSOR_CONFIG,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "POSITION",
            "options": [
              [
                {
                  "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/coordinate_1.png",
                  "width": 20,
                  "height": 20,
                  "alt": "Position 1"
                },
                "1"
              ],
              [
                {
                  "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/coordinate_2.png",
                  "width": 20,
                  "height": 20,
                  "alt": "Position 2"
                },
                "2"
              ],
              [
                {
                  "src": "https://ohstem-public.s3.ap-southeast-1.amazonaws.com/extensions/AITT-VN/yolouno_extension_robotics/images/coordinate_3.png",
                  "width": 20,
                  "height": 20,
                  "alt": "Position 3"
                },
                "3"
              ]
            ]
          }
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsSensorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_angle_sensor_config"] = function (block) {
  var position = block.getFieldValue("POSITION");
  var t = "(0, 1, 2)";
  var s = "(1, 1, 1)";
  
  if (position == "2") {
    t = "(2, 1, 0)";
    s = "(-1, -1, -1)";
  } else if (position == "3") {
    t = "(2, 0, 1)";
    s = "(-1, -1, 1)";
  }

  if (Blockly.Python.definitions_['init_robotics_mpu6050']) {
    Blockly.Python.definitions_['init_robotics_mpu6050'] = 'imu = MPU6050(transposition=' + t + ', scaling=' + s + ')';
  }
  if (Blockly.Python.definitions_['init_robotics_mpu9250']) {
    Blockly.Python.definitions_['init_robotics_mpu9250'] = 'imu = MPU9250(transposition=' + t + ', scaling=' + s + ')';
  }
  return '';
};

Blockly.Blocks["robotics_get_battery"] = {
  init: function () {
    this.jsonInit({
      colour: roboticsSensorBlockColor,
      tooltip: "",
      message0: Blockly.Msg.ROBOTICS_ROBOT_GET_BATTERY,
      args0: [],
      output: "Number",
      helpUrl: ""
    });
  },
};

Blockly.Python["robotics_get_battery"] = function (block) {
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_mdv2'] = 'from mdv2 import *';
  Blockly.Python.definitions_['init_motor_driver_v2'] = 'md_v2 = MotorDriverV2()';
  var code = "md_v2.battery()";
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks['robotics_angle_sensor_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_angle_sensor_init",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_ANGLE_SENSOR_INIT,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "type",
            "options": [
              ["MPU6050", "MPU6050"],
              ["MPU9250", "MPU9250"]
            ]
          },
          {
            type: "input_value",
            check: "Number",
            value: 100,
            name: "samples",
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsSensorBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

// Line sensor and line following

Blockly.Blocks['robotics_line_sensor_i2c_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_line_sensor_i2c_init",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_I2C_LINE_SENSOR_INIT,
        "args0": [],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsLineBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_line_sensor_i2c_init"] = function (block) {
  // TODO: Assemble Python into code variable.
  Blockly.Python.definitions_['import_robotics_line_sensor'] = 'from line_sensor import *';
  Blockly.Python.definitions_['init_robotics_line_sensor'] = 'line_sensor = LineSensorI2C()';
  var code = "robot.line_sensor(line_sensor)\n";
  return code;
};

Blockly.Blocks['robotics_line_sensor_digital_init'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_line_sensor_digital_init",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_DIGITAL_LINE_SENSOR_INIT,
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
        "colour": roboticsLineBlockColor,
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
  Blockly.Python.definitions_['import_robotics_line_sensor'] = 'from line_sensor import *';
  if (s3 == 'None') {
    Blockly.Python.definitions_['init_robotics_line_sensor'] = 
      'line_sensor = LineSensor2P(' + s1 + '_PIN, ' + s2 + '_PIN)';  
  } else {
    Blockly.Python.definitions_['init_robotics_line_sensor'] = 
      'line_sensor = LineSensor3P(' + s1 + '_PIN, ' + s2 + '_PIN, ' + s3 + '_PIN)';  
  }
  var code = "robot.line_sensor(line_sensor)\n";
  return code;
  
};
// Line array

Blockly.Blocks['robotics_line_sensor_read_all'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_line_sensor_read_all",
        "message0": Blockly.Msg.ROBOTICS_LINE_READ_ALL_MESSAGE0,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "S1",
            "options": [
              [
                {
                  "src": ImgUrl2 + 'line_finder_none_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "none"
                },
                "0"
              ],
              [
                {
                  "src": ImgUrl2 + 'line_finder_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "detect"
                },
                "1"
              ]
            ]
          },
          {
            "type": "field_dropdown",
            "name": "S2",
            "options": [
              [
                {
                  "src": ImgUrl2 + 'line_finder_none_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "none"
                },
                "0"
              ],
              [
                {
                  "src": ImgUrl2 + 'line_finder_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "detect"
                },
                "1"
              ]
            ]
          },
          {
            "type": "field_dropdown",
            "name": "S3",
            "options": [
              [
                {
                  "src": ImgUrl2 + 'line_finder_none_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "none"
                },
                "0"
              ],
              [
                {
                  "src": ImgUrl2 + 'line_finder_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "detect"
                },
                "1"
              ]
            ]
          },
          {
            "type": "field_dropdown",
            "name": "S4",
            "options": [
              [
                {
                  "src": ImgUrl2 + 'line_finder_none_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "none"
                },
                "0"
              ],
              [
                {
                  "src": ImgUrl2 + 'line_finder_detect.png',
                  "width": 15,
                  "height": 15,
                  "alt": "detect"
                },
                "1"
              ]
            ]
          }
        ],
        "colour": roboticsLineBlockColor,
        "output": "Boolean",
        "tooltip": Blockly.Msg.ROBOTICS_LINE_READ_ALL_TOOLTIP,
        "helpUrl": Blockly.Msg.ROBOTICS_LINE_READ_ALL_HELPURL
      }
    );
  }
};

Blockly.Python["robotics_line_sensor_read_all"] = function (block) {
  Blockly.Python.definitions_['import_line_sensor'] = 'from line_sensor import *';
  Blockly.Python.definitions_['init_robotics_line_sensor'] = 'line_sensor = LineSensorI2C()';
  var S1 = block.getFieldValue("S1");
  var S2 = block.getFieldValue("S2");
  var S3 = block.getFieldValue("S3");
  var S4 = block.getFieldValue("S4");
  // TODO: Assemble Python into code variable.
  var code = "line_sensor.read() == (" + S1 + ", " + S2 + ", " + S3 + ", " + S4 + ")";
  return [code, Blockly.Python.ORDER_NONE];
};

Blockly.Blocks['robotics_line_sensor_read'] = {
  init: function() {
    this.jsonInit(
      {
        "type": "robotics_line_sensor__read",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_LINE_SENSOR_READ,
        "args0": [
          {
            "type": "field_dropdown",
            "name": "port",
            "options": [
              ["S1", "0"],
              ["S2", "1"],
              ["S3", "2"],
              ["S4", "3"],
            ],
          },
        ],
        "colour": roboticsLineBlockColor,
        "output": "Boolean",
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_line_sensor_read"] = function (block) {
  var port = block.getFieldValue("port");
  // TODO: Assemble Python into code variable.
  var code = "line_sensor.read(" + port + ")";
  return [code, Blockly.Python.ORDER_NONE];
};

// ============================================================================
//  Line array 5 mat (STM32G030 I2C slave, dia chi 0x24)
// ============================================================================

// Tuy chon dropdown 0/1 (none/detect) dung chung cho cac mat.
function line5DetectOptions(name) {
  return {
    "type": "field_dropdown",
    "name": name,
    "options": [
      [{ "src": ImgUrl2 + 'line_finder_none_detect.png', "width": 15, "height": 15, "alt": "none" }, "0"],
      [{ "src": ImgUrl2 + 'line_finder_detect.png', "width": 15, "height": 15, "alt": "detect" }, "1"]
    ]
  };
}

// Dung CHUNG key voi khoi init 4 mat -> chi sinh dung 1 dong khoi tao.
// LineSensorI2C() tu detect 4 hay 5 mat theo dia chi I2C (xem line_sensor.py).
var _line5_init_defs = function () {
  Blockly.Python.definitions_['import_robotics_line_sensor'] = 'from line_sensor import *';
  Blockly.Python.definitions_['init_robotics_line_sensor'] = 'line_sensor = LineSensorI2C()';
};

// Cam bien mau VEML6040 doc lap voi cam bien do line: co bien color_sensor rieng
// (bus I2C rieng). Nguoi dung cam bien VEML6040 roi van dung duoc ma khong can ban 5 mat.
var _color_init_defs = function () {
  Blockly.Python.definitions_['import_robotics_color_sensor'] = 'from veml6040 import VEML6040';
  Blockly.Python.definitions_['init_robotics_color_sensor'] = 'color_sensor = VEML6040()';
};

Blockly.Blocks['robotics_line5_init'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_init",
      "message0": Blockly.Msg.ROBOTICS_ROBOT_I2C_LINE5_SENSOR_INIT,
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_init"] = function (block) {
  _line5_init_defs();
  // Dang ky voi drivebase de dung duoc cac khoi "do line ..." san co.
  var code = "robot.line_sensor(line_sensor)\n";
  return code;
};

Blockly.Blocks['robotics_line5_read_all'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_read_all",
      "message0": Blockly.Msg.ROBOTICS_LINE5_READ_ALL_MESSAGE0,
      "args0": [
        line5DetectOptions("S1"),
        line5DetectOptions("S2"),
        line5DetectOptions("S3"),
        line5DetectOptions("S4"),
        line5DetectOptions("S5")
      ],
      "colour": roboticsLineBlockColor,
      "output": "Boolean",
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_read_all"] = function (block) {
  _line5_init_defs();
  var S1 = block.getFieldValue("S1");
  var S2 = block.getFieldValue("S2");
  var S3 = block.getFieldValue("S3");
  var S4 = block.getFieldValue("S4");
  var S5 = block.getFieldValue("S5");
  var code = "line_sensor.read() == (" + S1 + ", " + S2 + ", " + S3 + ", " + S4 + ", " + S5 + ")";
  return [code, Blockly.Python.ORDER_NONE];
};


Blockly.Blocks['robotics_line5_read_raw'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_read_raw",
      "message0": Blockly.Msg.ROBOTICS_LINE5_READ_RAW,
      "args0": [
        {
          "type": "field_dropdown",
          "name": "port",
          "options": [
            [Blockly.Msg.ROBOTICS_LINE5_ALL || "All", "all"],
            ["S1", "0"], ["S2", "1"], ["S3", "2"], ["S4", "3"], ["S5", "4"]
          ]
        }
      ],
      "colour": roboticsLineBlockColor,
      "output": null,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_read_raw"] = function (block) {
  _line5_init_defs();
  var port = block.getFieldValue("port");
  var code = (port === "all") ? "line_sensor.read_raw()" : "line_sensor.read_raw(" + port + ")";
  return [code, Blockly.Python.ORDER_ATOMIC];
};


Blockly.Blocks['robotics_line5_read_mode'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_read_mode",
      "message0": Blockly.Msg.ROBOTICS_LINE5_READ_MODE,
      "args0": [
        {
          "type": "field_dropdown",
          "name": "mode",
          "options": [
            [Blockly.Msg.ROBOTICS_LINE5_MODE_DIGITAL || "digital", "digital"],
            [Blockly.Msg.ROBOTICS_LINE5_MODE_ANALOG  || "analog",  "analog"]
          ]
        },
        {
          "type": "field_dropdown",
          "name": "port",
          "options": [
            [Blockly.Msg.ROBOTICS_LINE5_ALL || "tất cả", "all"],
            ["S1", "0"], ["S2", "1"], ["S3", "2"], ["S4", "3"], ["S5", "4"]
          ]
        }
      ],
      "colour": roboticsLineBlockColor,
      "output": null,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_read_mode"] = function (block) {
  _line5_init_defs();
  var mode = block.getFieldValue("mode");
  var port = block.getFieldValue("port");
  var method = (mode === "analog") ? "read_raw" : "read";
  var code = (port === "all")
    ? "line_sensor." + method + "()"
    : "line_sensor." + method + "(" + port + ")";
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Blocks['robotics_line5_position'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_position",
      "message0": Blockly.Msg.ROBOTICS_LINE5_POSITION,
      "args0": [],
      "colour": roboticsLineBlockColor,
      "output": "Number",
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_position"] = function (block) {
  _line5_init_defs();
  var code = "line_sensor.position()";
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Blocks['robotics_line5_set_white_led'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_set_white_led",
      "message0": Blockly.Msg.ROBOTICS_LINE5_SET_WHITE_LED,
      "args0": [
        {
          "type": "field_dropdown",
          "name": "state",
          "options": [[Blockly.Msg.ROBOTICS_ON || "ON", "True"], [Blockly.Msg.ROBOTICS_OFF || "OFF", "False"]]
        }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_set_white_led"] = function (block) {
  _line5_init_defs();
  var state = block.getFieldValue("state");
  var code = "line_sensor.set_white_led(" + state + ")\n";
  return code;
};

Blockly.Blocks['robotics_line5_calibrate'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_calibrate",
      "message0": Blockly.Msg.ROBOTICS_LINE5_CALIBRATE,
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_calibrate"] = function (block) {
  _line5_init_defs();
  var code = "line_sensor.calibrate()\n";
  return code;
};

// ============================================================================
//  Cam bien mau VEML6040 (doc lap voi cam bien do line -> bien color_sensor rieng).
//  Nguoi dung VEML6040 roi van dung duoc; khong phu thuoc ban line 5 mat.
//  (Rieng LED trang van thuoc cam bien line 5 mat: robotics_line5_set_white_led.)
// ============================================================================
Blockly.Blocks['robotics_color_start'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_color_start",
      "message0": Blockly.Msg.ROBOTICS_COLOR_START || "bật xử lý cảm biến màu",
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsSensorBlockColor,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_color_start"] = function (block) {
  _color_init_defs();
  return "create_task(color_sensor.color_run())\n";
};

Blockly.Blocks['robotics_color_detect'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_color_detect",
      "message0": Blockly.Msg.ROBOTICS_COLOR_DETECT || "cảm biến màu phát hiện màu %1",
      "args0": [
        {
          "type": "field_dropdown",
          "name": "COLOR",
          "options": [
            ["vàng", "yellow"],
            ["đỏ", "red"],
            ["xanh lá", "green"],
            ["xanh lơ", "cyan"],
            ["xanh dương", "blue"],
            ["hồng thẫm", "magenta"]
          ]
        }
      ],
      "colour": roboticsSensorBlockColor,
      "output": "Boolean",
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_color_detect"] = function (block) {
  _color_init_defs();
  var color = block.getFieldValue("COLOR");
  var code = '(color_sensor.color() == "' + color + '")';
  return [code, Blockly.Python.ORDER_ATOMIC];
};

Blockly.Blocks['robotics_color_read'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_color_read",
      "message0": Blockly.Msg.ROBOTICS_COLOR_READ || "cảm biến màu đọc %1",
      "args0": [
        {
          "type": "field_dropdown",
          "name": "VALUE",
          "options": [
            ["độ sáng (lux)", "LUX"],
            ["giá trị đỏ", "RED"],
            ["giá trị xanh lá", "GREEN"],
            ["giá trị xanh dương", "BLUE"],
            ["nhiệt độ màu", "CCT"]
          ]
        }
      ],
      "colour": roboticsSensorBlockColor,
      "output": "Number",
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_color_read"] = function (block) {
  _color_init_defs();
  var value = block.getFieldValue("VALUE");
  var code;
  if (value === 'LUX') {
    code = 'color_sensor.get_lux()';
  } else if (value === 'CCT') {
    code = 'color_sensor.get_cct()';
  } else {
    code = 'color_sensor.get_' + value.toLowerCase() + '()';
  }
  return [code, Blockly.Python.ORDER_ATOMIC];
};

// Hieu chuan tham chieu 1 mau: dat cam bien len be mat mau roi chon mau tuong ung.
// "nen" (background) -> do lai tham chieu nen trang (_veml refs 'white' -> phan loai None).
Blockly.Blocks['robotics_color_calibrate'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_color_calibrate",
      "message0": Blockly.Msg.ROBOTICS_COLOR_CALIBRATE || "hiệu chuẩn màu %1",
      "args0": [
        {
          "type": "field_dropdown",
          "name": "COLOR",
          "options": [
            [Blockly.Msg.ROBOTICS_COLOR_BACKGROUND || "nền", "background"],
            ["đỏ", "red"],
            ["vàng", "yellow"],
            ["xanh lá", "green"],
            ["xanh lơ", "cyan"],
            ["xanh dương", "blue"],
            ["hồng thẫm", "magenta"]
          ]
        }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsSensorBlockColor,
      "tooltip": "",
      "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_color_calibrate"] = function (block) {
  _color_init_defs();
  var color = block.getFieldValue("COLOR");
  // "nen" map sang tham chieu 'white' cua VEML (mau nen -> phan loai None).
  var name = (color === "background") ? "white" : color;
  return 'color_sensor.calibrate_color("' + name + '")\n';
};

Blockly.Blocks['robotics_follow_line_until_cross'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_until_cross",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_FOLLOW_LINE_UNTIL_CROSS,
        "args0": [
          {
            type: "field_dropdown",
            name: "stop",
            options: [
            [Blockly.Msg.ROBOTICS_BRAKE , "BRAKE"],
            [Blockly.Msg.ROBOTICS_STOP, "STOP"],
            [Blockly.Msg.ROBOTICS_NONE, "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsLineBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_until_cross"] = function (block) {
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_until_cross(then=" + stop +  ")\n";
  return code;
};

Blockly.Blocks['robotics_follow_line_until_end'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_until_end",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_FOLLOW_END_LINE,
        "args0": [
          {
            type: "field_dropdown",
            name: "stop",
            options: [
              [Blockly.Msg.ROBOTICS_BRAKE , "BRAKE"],
              [Blockly.Msg.ROBOTICS_STOP, "STOP"],
              [Blockly.Msg.ROBOTICS_NONE, "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsLineBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_until_end"] = function (block) {
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_until_end(then=" + stop +  ")\n";
  return code;
};

Blockly.Blocks['robotics_turn_until_line_detected_then'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_turn_until_line_detected_then",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_TURN_UNTIL_LINE_DETECTED,
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
            type: "field_dropdown",
            name: "stop",
            options: [
              [Blockly.Msg.ROBOTICS_BRAKE , "BRAKE"],
              [Blockly.Msg.ROBOTICS_STOP, "STOP"],
              [Blockly.Msg.ROBOTICS_NONE, "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsLineBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_turn_until_line_detected_then"] = function (block) {
  var dir = block.getFieldValue('direction');
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "";
  if (dir == "left") {
    code = "await robot.turn_until_line_detected(steering=-100, then=" + stop + ")\n";
  } else {
    code = "await robot.turn_until_line_detected(steering=100, then=" + stop +  ")\n";
  }
  return code;
};

Blockly.Blocks['robotics_follow_line_by_time'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_by_time",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_FOLLOW_LINE_BY_TIME,
        "args0": [
          {
            type: "input_value",
            check: "Number",
            value: 1,
            name: "duration",
          },
          {
            type: "field_dropdown",
            name: "stop",
            options: [
              [Blockly.Msg.ROBOTICS_BRAKE , "BRAKE"],
              [Blockly.Msg.ROBOTICS_STOP, "STOP"],
              [Blockly.Msg.ROBOTICS_NONE, "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsLineBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_by_time"] = function (block) {
  var duration = Blockly.Python.valueToCode(block, 'duration', Blockly.Python.ORDER_ATOMIC);
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_by_time(" + duration + ", then=" + stop + ")\n";
  return code;
};


Blockly.Blocks['robotics_follow_line_until'] = {
  init: function () {
    this.jsonInit(
      {
        "type": "robotics_follow_line_until",
        "message0": Blockly.Msg.ROBOTICS_ROBOT_FOLLOW_LINE_UNTIL,
        "args0": [
          {
            "type": "input_value",
            "name": "condition",
          },
          {
            type: "field_dropdown",
            name: "stop",
            options: [
              [Blockly.Msg.ROBOTICS_BRAKE , "BRAKE"],
              [Blockly.Msg.ROBOTICS_STOP, "STOP"],
              [Blockly.Msg.ROBOTICS_NONE, "None"],
            ]
          },
        ],
        "inputsInline": true,
        "previousStatement": null,
        "nextStatement": null,
        "colour": roboticsLineBlockColor,
        "tooltip": "",
        "helpUrl": ""
      }
    );
  }
};

Blockly.Python["robotics_follow_line_until"] = function (block) {
  var condition = Blockly.Python.valueToCode(block, 'condition', Blockly.Python.ORDER_ATOMIC);
  var stop = block.getFieldValue('stop');
  // TODO: Assemble Python into code variable.
  var code = "await robot.follow_line_until(" + "lambda: " + condition + ", then=" + stop + ")\n";
  return code;
};

// ============================================================================
//  Line array 5 mat V2 - PID bam line theo centroid + FSM xu ly checkpoint
// ============================================================================

// ---- BLOCK thiet lap: PID do line (Kp/Ki/Kd) ----
Blockly.Blocks['robotics_line5_set_pid'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_set_pid",
      "message0": Blockly.Msg.ROBOTICS_LINE5_SET_PID,
      "args0": [
        { type: "input_value", check: "Number", name: "KP" },
        { type: "input_value", check: "Number", name: "KI" },
        { type: "input_value", check: "Number", name: "KD" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_set_pid"] = function (block) {
  var kp = Blockly.Python.valueToCode(block, 'KP', Blockly.Python.ORDER_ATOMIC);
  var ki = Blockly.Python.valueToCode(block, 'KI', Blockly.Python.ORDER_ATOMIC);
  var kd = Blockly.Python.valueToCode(block, 'KD', Blockly.Python.ORDER_ATOMIC);
  var code = "robot.line_pid(" + kp + ", " + ki + ", " + kd + ")\n";
  return code;
};

// ---- BLOCK thiet lap: dac tinh toc do khi do line ----
Blockly.Blocks['robotics_line5_set_line_speed'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_set_line_speed",
      "message0": Blockly.Msg.ROBOTICS_LINE5_SET_LINE_SPEED,
      "args0": [
        { type: "input_value", check: "Number", name: "MIN_RATIO" },
        {
          "type": "field_dropdown",
          "name": "INVERT",
          "options": [
            [Blockly.Msg.ROBOTICS_LINE5_INVERT_NORMAL || "normal", "1"],
            [Blockly.Msg.ROBOTICS_LINE5_INVERT_REVERSED || "reversed", "-1"]
          ]
        }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_set_line_speed"] = function (block) {
  var ratio = Blockly.Python.valueToCode(block, 'MIN_RATIO', Blockly.Python.ORDER_ATOMIC);
  var invert = block.getFieldValue("INVERT");
  var code = "robot.line_curve_gain(" + ratio + ")\nrobot.line_invert(" + invert + ")\n";
  return code;
};

// ---- BLOCK thiet lap: loc nhieu (debounce) checkpoint ----
Blockly.Blocks['robotics_line5_set_debounce'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_set_debounce",
      "message0": Blockly.Msg.ROBOTICS_LINE5_SET_DEBOUNCE,
      "args0": [
        { type: "input_value", check: "Number", name: "FRAMES" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_set_debounce"] = function (block) {
  _line5_init_defs();
  var frames = Blockly.Python.valueToCode(block, 'FRAMES', Blockly.Python.ORDER_ATOMIC);
  var code = "line_sensor.set_debounce(" + frames + ")\n";
  return code;
};

// ---- BLOCK chay: do line PID tu dong + xu ly checkpoint (vong lap FSM) ----
Blockly.Blocks['robotics_line5_follow_run'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_follow_run",
      "message0": Blockly.Msg.ROBOTICS_LINE5_FOLLOW_RUN,
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_follow_run"] = function (block) {
  _line5_init_defs();
  var code = "await robot.run_line_follow()\n";
  return code;
};

// ---- BLOCK chay: do line PID mot buoc (dung trong vong lap tu viet) ----
Blockly.Blocks['robotics_line5_follow_step'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_follow_step",
      "message0": Blockly.Msg.ROBOTICS_LINE5_FOLLOW_STEP,
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_follow_step"] = function (block) {
  _line5_init_defs();
  var code = "robot.follow_line_pid()\n";
  return code;
};

// ---- BLOCK doc: cap nhat cam bien 1 lan (goi truoc khi doc pattern/error/checkpoint) ----
Blockly.Blocks['robotics_line5_update'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_update",
      "message0": Blockly.Msg.ROBOTICS_LINE5_UPDATE,
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_update"] = function (block) {
  _line5_init_defs();
  var code = "line_sensor.update()\n";
  return code;
};

// ---- BLOCK doc (reporter): bit pattern 5 mat ----
Blockly.Blocks['robotics_line5_get_pattern'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_get_pattern",
      "message0": Blockly.Msg.ROBOTICS_LINE5_GET_PATTERN,
      "args0": [],
      "colour": roboticsLineBlockColor,
      "output": "Number",
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_get_pattern"] = function (block) {
  _line5_init_defs();
  return ["line_sensor.get_pattern()", Blockly.Python.ORDER_ATOMIC];
};

// ---- BLOCK doc (reporter): sai so line cho PID [-2000..2000] ----
Blockly.Blocks['robotics_line5_get_error'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_get_error",
      "message0": Blockly.Msg.ROBOTICS_LINE5_GET_ERROR,
      "args0": [],
      "colour": roboticsLineBlockColor,
      "output": "Number",
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_get_error"] = function (block) {
  _line5_init_defs();
  return ["line_sensor.get_error()", Blockly.Python.ORDER_ATOMIC];
};

// ---- BLOCK doc (boolean): checkpoint hien tai == ? ----
Blockly.Blocks['robotics_line5_checkpoint_is'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_line5_checkpoint_is",
      "message0": Blockly.Msg.ROBOTICS_LINE5_CHECKPOINT_IS,
      "args0": [
        {
          "type": "field_dropdown",
          "name": "CP",
          "options": [
            [Blockly.Msg.ROBOTICS_LINE5_CP_NORMAL || "straight",     "LINE_NORMAL"],
            [Blockly.Msg.ROBOTICS_LINE5_CP_LEFT   || "left corner",  "LINE_LEFT_CORNER"],
            [Blockly.Msg.ROBOTICS_LINE5_CP_RIGHT  || "right corner", "LINE_RIGHT_CORNER"],
            [Blockly.Msg.ROBOTICS_LINE5_CP_CROSS  || "cross",        "LINE_CROSS"],
            [Blockly.Msg.ROBOTICS_LINE5_CP_Y      || "Y fork",       "LINE_Y"],
            [Blockly.Msg.ROBOTICS_LINE5_CP_LOST   || "lost line",    "LINE_LOST"]
          ]
        }
      ],
      "inputsInline": true,
      "colour": roboticsLineBlockColor,
      "output": "Boolean",
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_line5_checkpoint_is"] = function (block) {
  _line5_init_defs();
  var cp = block.getFieldValue("CP");
  return ["line_sensor.detect_checkpoint() == " + cp, Blockly.Python.ORDER_RELATIONAL];
};


// ============================================================================
//  LINE PID - do line 5 mat / 4 mat toc do cao (engine da merge vao DriveBase).
//  Dung lai DriveBase (robot) + cam bien "line_sensor" da dang ky. Cac khoi cau
//  hinh sinh robot.line_*(...). Viec "do line den..." dung CHUNG khoi Check point.
// ============================================================================

// 1) Chon che do (digital / raw)
Blockly.Blocks['robotics_fast_line_enable'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_enable",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_ENABLE,
      "args0": [
        {
          "type": "field_dropdown",
          "name": "mode",
          "options": [
            [Blockly.Msg.ROBOTICS_FAST_LINE_MODE_DIGITAL || "digital", "digital"],
            [Blockly.Msg.ROBOTICS_FAST_LINE_MODE_RAW || "raw", "raw"]
          ]
        }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_enable"] = function (block) {
  var mode = block.getFieldValue("mode");
  return "robot.line_mode('" + mode + "')\n";
};

// 2) Calibrate (cho che do raw, chi cam bien 5 mat)
Blockly.Blocks['robotics_fast_line_calibrate'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_calibrate",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_CALIBRATE,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "seconds" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_calibrate"] = function (block) {
  var seconds = Blockly.Python.valueToCode(block, 'seconds', Blockly.Python.ORDER_ATOMIC) || '3';
  return "await robot.line_calibrate(" + seconds + ")\n";
};

// 3) Dat he so PID
Blockly.Blocks['robotics_fast_line_set_pid'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_pid",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_PID,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "kp" },
        { "type": "input_value", "check": "Number", "name": "ki" },
        { "type": "input_value", "check": "Number", "name": "kd" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_pid"] = function (block) {
  var kp = Blockly.Python.valueToCode(block, 'kp', Blockly.Python.ORDER_ATOMIC) || '0';
  var ki = Blockly.Python.valueToCode(block, 'ki', Blockly.Python.ORDER_ATOMIC) || '0';
  var kd = Blockly.Python.valueToCode(block, 'kd', Blockly.Python.ORDER_ATOMIC) || '0';
  return "robot.line_pid(" + kp + ", " + ki + ", " + kd + ")\n";
};

// 4) Dat toc do
Blockly.Blocks['robotics_fast_line_set_speed'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_speed",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_SPEED,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "speed" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_speed"] = function (block) {
  var speed = Blockly.Python.valueToCode(block, 'speed', Blockly.Python.ORDER_ATOMIC) || '60';
  return "robot.line_speed(" + speed + ")\n";
};

// 4b) Dat toc do toi thieu / toi da (san & tran RIENG cho do line, doc lap robot.speed)
Blockly.Blocks['robotics_fast_line_set_speed_range'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_speed_range",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_SPEED_RANGE,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "min" },
        { "type": "input_value", "check": "Number", "name": "max" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_speed_range"] = function (block) {
  var min = Blockly.Python.valueToCode(block, 'min', Blockly.Python.ORDER_ATOMIC) || '40';
  var max = Blockly.Python.valueToCode(block, 'max', Blockly.Python.ORDER_ATOMIC) || '60';
  return "robot.line_speed(min_speed=" + min + ", max_speed=" + max + ")\n";
};

// 5) Dat do giam toc khi cua (curve_gain)
Blockly.Blocks['robotics_fast_line_set_curve_gain'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_curve_gain",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_CURVE_GAIN,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "gain" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_curve_gain"] = function (block) {
  var gain = Blockly.Python.valueToCode(block, 'gain', Blockly.Python.ORDER_ATOMIC) || '0.7';
  return "robot.line_curve_gain(" + gain + ")\n";
};

// 5b) Bu offset cam bien khi quay: do line tien them 'sec' giay truoc khi quay
Blockly.Blocks['robotics_fast_line_set_turn_offset'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_turn_offset",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_TURN_OFFSET,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "sec" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_turn_offset"] = function (block) {
  var sec = Blockly.Python.valueToCode(block, 'sec', Blockly.Python.ORDER_ATOMIC) || '0';
  return "robot.line_turn_offset(" + sec + ")\n";
};

// 6) Bat/tat debug + khoang in (CSV de tinh chinh PID)
Blockly.Blocks['robotics_fast_line_debug'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_debug",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_DEBUG,
      "args0": [
        {
          "type": "field_dropdown",
          "name": "state",
          "options": [
            [Blockly.Msg.ROBOTICS_ON || "on", "True"],
            [Blockly.Msg.ROBOTICS_OFF || "off", "False"]
          ]
        },
        { "type": "input_value", "check": "Number", "name": "ms" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_debug"] = function (block) {
  var state = block.getFieldValue("state");
  var ms = Blockly.Python.valueToCode(block, 'ms', Blockly.Python.ORDER_ATOMIC) || '100';
  return "robot.line_debug_interval(" + ms + ")\nrobot.line_debug(" + state + ")\n";
};

// 10) Mot buoc PID (de tu ghep vong lap)
Blockly.Blocks['robotics_fast_line_step'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_step",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_STEP,
      "args0": [],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_step"] = function (block) {
  return "robot.follow_line_pid()\n";
};

// Cac khoi "do line den..." (follow_delay / follow_until_cross / follow_until / stop)
// da GO khoi nhom nay -> dung CHUNG khoi Check point (robot.follow_line_*).

// 15) Debounce vach ngang (tranh phat hien sai tai cua gat)
Blockly.Blocks['robotics_fast_line_set_cross_debounce'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_cross_debounce",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_CROSS_DEBOUNCE,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "frames" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": "#34ccf1"
    });
  }
};

Blockly.Python["robotics_fast_line_set_cross_debounce"] = function (block) {
  var frames = Blockly.Python.valueToCode(block, 'frames', Blockly.Python.ORDER_ATOMIC) || '5';
  return "robot.line_cross_debounce(" + frames + ")\n";
};

// 8) Luc lai + gioi han correction (turn_gain)
Blockly.Blocks['robotics_fast_line_set_turn_gain'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_turn_gain",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_TURN_GAIN,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "gain" },
        { "type": "input_value", "check": "Number", "name": "limit" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_turn_gain"] = function (block) {
  var gain = Blockly.Python.valueToCode(block, 'gain', Blockly.Python.ORDER_ATOMIC) || '0.8';
  var limit = Blockly.Python.valueToCode(block, 'limit', Blockly.Python.ORDER_ATOMIC) || '1.0';
  return "robot.line_turn_gain(" + gain + ", correction_limit=" + limit + ")\n";
};

// 9) Vung chet (deadband) - |error| <= db coi nhu di thang tap
Blockly.Blocks['robotics_fast_line_set_deadband'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_deadband",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_DEADBAND,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "db" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_deadband"] = function (block) {
  var db = Blockly.Python.valueToCode(block, 'db', Blockly.Python.ORDER_ATOMIC) || '0.3';
  return "robot.line_deadband(" + db + ")\n";
};

// 10) San bu ma sat (stall_floor) - 0 = om cua em
Blockly.Blocks['robotics_fast_line_set_stall_floor'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_stall_floor",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_STALL_FLOOR,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "floor" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_stall_floor"] = function (block) {
  var floor = Blockly.Python.valueToCode(block, 'floor', Blockly.Python.ORDER_ATOMIC) || '0';
  return "robot.line_stall_floor(" + floor + ")\n";
};

// 11) Loc khau D (d_alpha)
Blockly.Blocks['robotics_fast_line_set_d_alpha'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_d_alpha",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_D_ALPHA,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "alpha" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_d_alpha"] = function (block) {
  var alpha = Blockly.Python.valueToCode(block, 'alpha', Blockly.Python.ORDER_ATOMIC) || '0.5';
  return "robot.line_d_alpha(" + alpha + ")\n";
};

// 12) Loc error dau vao (ema_alpha)
Blockly.Blocks['robotics_fast_line_set_ema_alpha'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_ema_alpha",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_EMA_ALPHA,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "alpha" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_ema_alpha"] = function (block) {
  var alpha = Blockly.Python.valueToCode(block, 'alpha', Blockly.Python.ORDER_ATOMIC) || '0.5';
  return "robot.line_ema_alpha(" + alpha + ")\n";
};

// 13) Toc tien giu lai khi mat line (lost_fwd)
Blockly.Blocks['robotics_fast_line_set_lost_fwd'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_set_lost_fwd",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_SET_LOST_FWD,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "ratio" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_set_lost_fwd"] = function (block) {
  var ratio = Blockly.Python.valueToCode(block, 'ratio', Blockly.Python.ORDER_ATOMIC) || '0.2';
  return "robot.line_lost_fwd(" + ratio + ")\n";
};

Blockly.Blocks['robotics_fast_line_quick_setup'] = {
  init: function () {
    this.jsonInit({
      "type": "robotics_fast_line_quick_setup",
      "message0": Blockly.Msg.ROBOTICS_FAST_LINE_QUICK_SETUP,
      "args0": [
        { "type": "input_value", "check": "Number", "name": "kp" },
        { "type": "input_value", "check": "Number", "name": "ki" },
        { "type": "input_value", "check": "Number", "name": "kd" }
      ],
      "inputsInline": true,
      "previousStatement": null,
      "nextStatement": null,
      "colour": roboticsLineBlockColor,
      "tooltip": "", "helpUrl": ""
    });
  }
};

Blockly.Python["robotics_fast_line_quick_setup"] = function (block) {
  var kp    = Blockly.Python.valueToCode(block, 'kp',    Blockly.Python.ORDER_ATOMIC) || '1.5';
  var ki    = Blockly.Python.valueToCode(block, 'ki',    Blockly.Python.ORDER_ATOMIC) || '0';
  var kd    = Blockly.Python.valueToCode(block, 'kd',    Blockly.Python.ORDER_ATOMIC) || '16';
  return (
    "robot.line_pid(" + kp + ", " + ki + ", " + kd + ")\n" +
    "robot.line_turn_gain(0.6, correction_limit=1)\n" +
    "robot.line_deadband(0.3)\n" +
    "robot.line_d_alpha(0.5)\n" +
    "robot.line_lost_fwd(0.2)\n"
  );
};
