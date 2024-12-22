%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2017 ROBOTIS CO., LTD.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Read and Write Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 2.0
% This example is designed for using a Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
%

% clc;
% clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address
MX28_TORQUE_ENABLE        = 64;             % Control table address is different in Dynamixel model
MX28_GOAL_POSITION        = 116;
% MX28_PRESENT_POSITION     = 132;
% MX28_POSITION_P           = 84;
% MX28_POSITION_D           = 80;

% Protocol version
PROTOCOL_VERSION          = 2.0;            % See which protocol version is used in the Dynamixel

% Default setting
MX28_ID                   = [1 2 3 4 5 6];  % Dynamixel ID: 1 2 3 4 5 6
BAUDRATE                  = 57600;
DEVICENAME                = 'COM3';         % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE             = 1;              % Value for enabling the torque
TORQUE_DISABLE            = 0;              % Value for disabling the torque
% MINIMUM_POSITION_VALUE    = 0;              % Dynamixel will rotate between this value
% MAXIMUM_POSITION_VALUE    = 4095;           % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
% MOVING_STATUS_THRESHOLD   = 10;             % Dynamixel moving status threshold

% ESC_CHARACTER             = 'e';            % Key for escaping loop

COMM_SUCCESS              = 0;              % Communication Success result value
COMM_TX_FAIL              = -1001;          % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
% dxl_goal_position = [MINIMUM_POSITION_VALUE MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
% dxl_present_position = 0;                   % Present position

% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Enable Dynamixel Torque
for i = 1:6
    write1ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), MX28_TORQUE_ENABLE, TORQUE_ENABLE);
%     write2ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), MX28_POSITION_P, 700);
%     write2ByteTxRx(port_num, PROTOCOL_VERSION, MX28_ID(i), MX28_POSITION_D, 0);
end
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end