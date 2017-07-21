/*
* core.h
* Combined header of rpos core module
*
* Created by Tony Huang (cnwzhjs@gmail.com) at 2014-05-25
* Copyright 2012 (c) www.robopeak.com
*/

#pragma once

#include <rpos/rpos_config.h>

// Communication Abstraction
#include "channel.h"
#include "endpoint.h"
#include "message.h"
#include "message_payload.h"
#include "protocol.h"
#include "router.h"

// Device management
#include "device.h"
#include "device_manager.h"

// Robot platform
#include "robot_platform.h"

// Robotology core objects
#include "action.h"
#include "feature.h"
#include "pose.h"
#include "geometry.h"
#include "diagnosis_info.h"
