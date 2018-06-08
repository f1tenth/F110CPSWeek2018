/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_ROS_H
#define ROSCPP_ROS_H

#include "ros/time.h"
#include "ros/rate.h"
#include "ros/console.h"
#include "ros/assert.h"

#include "ros/common.h"
#include "ros/types.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/single_subscriber_publisher.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include "ros/service.h"
#include "ros/init.h"
#include "ros/master.h"
#include "ros/this_node.h"
#include "ros/param.h"
#include "ros/topic.h"
#include "ros/names.h"

/*
USAGE:
define TIME_PROFILE con cmake o scommentando qui sotto

per prendere un tempo:
	PTIME_INIT()   		//prima di tutto inizializzare le variabili
	PTIME_START()  		//inizio timer
	...
	PTIME_END()    		//fine timer
	PTIME_STAMP(,NOME) 	//stampa timer (la virgola serve!!)
	
è possibile assegnare un id al timer per poterne usare più di uno:
	PTIME_INIT(1)
	PTIME_INIT(2)
	PTIME_START(1)
	...
	PTIME_START(2)
	...
	PTIME_END(2)
	PTIME_STAMP(2, NESTED_TIMER)
	...
	PTIME_END(1)
	PTIME_STAMP(1, GLOBAL_TIMER)
	

#ifndef TIME_PROFILER
#define TIME_PROFILER
#endif
*/

#ifdef TIME_PROFILER
	#define PTIME_INIT(id) ros::WallTime p_start##id; \
			               ros::WallTime p_end##id;
	#define PTIME_START(id)    p_start##id = ros::WallTime::now();
	#define PTIME_END(id)      p_end##id = ros::WallTime::now(); 
	#define PTIME_STAMP(id, N) std::cout<<#N<<" "<<ros::Time::now()<<" "<<p_end##id-p_start##id<<std::endl;          
    #define PTIME_NOW(N)       std::cout<<#N<<" "<<ros::Time::now()<<std::endl;
#else
	#define PTIME_INIT(id)     ;
	#define PTIME_START(id)    ;
	#define PTIME_END(id)      ;
	#define PTIME_STAMP(id, N) ;
    #define PTIME_NOW(N)       ;
#endif

#endif
