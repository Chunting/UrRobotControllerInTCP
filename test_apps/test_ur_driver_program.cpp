//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include <string>

#define MULT_JOINTSTATE_ 1000000

void dump_ur_driver_prog(){
    std::string cmd_str;
    char buf[128];
    cmd_str = "def driverProg():\n";

    sprintf(buf, "\tMULT_jointstate = %i\n", MULT_JOINTSTATE_);
    cmd_str += buf;

    cmd_str += "\tSERVO_IDLE = 0\n";
    cmd_str += "\tSERVO_RUNNING = 1\n";
    cmd_str += "\tcmd_servo_state = SERVO_IDLE\n";
    cmd_str += "\tcmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
    cmd_str += "\tdef set_servo_setpoint(q):\n";
    cmd_str += "\t\tenter_critical\n";
    cmd_str += "\t\tcmd_servo_state = SERVO_RUNNING\n";
    cmd_str += "\t\tcmd_servo_q = q\n";
    cmd_str += "\t\texit_critical\n";
    cmd_str += "\tend\n";
    cmd_str += "\tthread servoThread():\n";
    cmd_str += "\t\tstate = SERVO_IDLE\n";
    cmd_str += "\t\twhile True:\n";
    cmd_str += "\t\t\tenter_critical\n";
    cmd_str += "\t\t\tq = cmd_servo_q\n";
    cmd_str += "\t\t\tdo_brake = False\n";
    cmd_str += "\t\t\tif (state == SERVO_RUNNING) and ";
    cmd_str += "(cmd_servo_state == SERVO_IDLE):\n";
    cmd_str += "\t\t\t\tdo_brake = True\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\t\tstate = cmd_servo_state\n";
    cmd_str += "\t\t\tcmd_servo_state = SERVO_IDLE\n";
    cmd_str += "\t\t\texit_critical\n";
    cmd_str += "\t\t\tif do_brake:\n";
    cmd_str += "\t\t\t\tstopj(1.0)\n";
    cmd_str += "\t\t\t\tsync()\n";
    cmd_str += "\t\t\telif state == SERVO_RUNNING:\n";

    if (5 >= 3.1)
        sprintf(buf, "\t\t\t\tservoj(q, t=%.4f, lookahead_time=%.4f, gain=%.0f)\n",
                5.5f, 1.5f, 1.1f);
    else
        sprintf(buf, "\t\t\t\tservoj(q, t=%.4f)\n", 5.5f);
    cmd_str += buf;

    cmd_str += "\t\t\telse:\n";
    cmd_str += "\t\t\t\tsync()\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\tend\n";

    sprintf(buf, "\tsocket_open(\"%s\", %i)\n", "ip_adress", 12345);
    cmd_str += buf;

    cmd_str += "\tthread_servo = run servoThread()\n";
    cmd_str += "\tkeepalive = 1\n";
    cmd_str += "\twhile keepalive > 0:\n";
    cmd_str += "\t\tparams_mult = socket_read_binary_integer(6+1)\n";
    cmd_str += "\t\tif params_mult[0] > 0:\n";
    cmd_str += "\t\t\tq = [params_mult[1] / MULT_jointstate, ";
    cmd_str += "params_mult[2] / MULT_jointstate, ";
    cmd_str += "params_mult[3] / MULT_jointstate, ";
    cmd_str += "params_mult[4] / MULT_jointstate, ";
    cmd_str += "params_mult[5] / MULT_jointstate, ";
    cmd_str += "params_mult[6] / MULT_jointstate]\n";
    cmd_str += "\t\t\tkeepalive = params_mult[7]\n";
    cmd_str += "\t\t\tset_servo_setpoint(q)\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\tend\n";
    cmd_str += "\tsleep(.1)\n";
    cmd_str += "\tsocket_close()\n";
    cmd_str += "\tkill thread_servo\n";
    cmd_str += "end\n";

    printf("%s", cmd_str.c_str());
}

int main(int argc, char** argv){
    dump_ur_driver_prog();
    return 0;
}