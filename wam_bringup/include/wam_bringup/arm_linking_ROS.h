/*
 * ex11_master_master.h
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 */

#ifndef ARM_LINKING_ROS_H_
#define ARM_LINKING_ROS_H_


#include <stdexcept>

#include <syslog.h>
#include <unistd.h> /* for close() */
#include <sys/socket.h> /* For sockets */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <arpa/inet.h>  /* For inet_pton() */

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/disable_secondary_mode_warning.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/single_io.h>

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

using namespace barrett;
using namespace systems;

template <size_t DOF>
class MasterMasterROS : public systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	//IO (inputs)
 public:
	Input<jp_type> JpInput;
	Input<jp_type> JpHandTool;
	//IO (output)
 public:
	Output<jp_type> JpOutput;

 protected:
       	typename Output<jp_type>::Value* jpOutputValue;

	/*class MasterMaster : public barrett::systems::SingleIO<typename barrett::units::JointPositions<DOF>::type, typename barrett::units::JointPositions<DOF>::type> {
	  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);*/

public:
    ros::NodeHandle n_; // WAM specific nodehandle
    ros::Subscriber joint_pos_sub;
	ros::Publisher wam_joint_state_pub;
	sensor_msgs::JointState wam_joint_state;


    void joint_pos_cb(const sensor_msgs::JointState _msg){

        int num_received = _msg.position.size();

		//ROS_INFO("data received: %i", num_received);
		//ROS_INFO("DOFs: %i", (int)DOF);

        for(int i=0; i<DOF; i++){
            theirJp[i] = _msg.position[i];
        }

        if(num_received>=(int)DOF){
			numMissed = 0;
			//ROS_INFO("numMissed: %i", numMissed);
        } else if(num_received==4 && DOF == 7){
            numMissed = 0;
			//ROS_INFO("numMissed: %i", numMissed);
            theirJp[4]=this->JpInput.getValue()[4];
            theirJp[5]=this->JpInput.getValue()[5];
            theirJp[6]=this->JpInput.getValue()[6];

        }

    }

	explicit MasterMasterROS(barrett::systems::ExecutionManager* em, char* remoteHost, int port = 5555, const std::string& sysName = "MasterMasterROS") :
	System(sysName),JpInput(this),JpHandTool(this),JpOutput(this,&jpOutputValue), sock(-1), linked(false), numMissed(NUM_MISSED_LIMIT), theirJp(0.0)
	{
		//int err;
		//long flags;
		//int buflen;
		//unsigned int buflenlen;
		//struct sockaddr_in bind_addr;
        //struct sockaddr_in their_addr;

        //
		const char* wam_jnts[] = {  "wam/YawJoint",
									"wam/ShoulderPitchJoint",
									"wam/ShoulderYawJoint",
									"wam/ElbowJoint",
									"wam/UpperWristYawJoint",
									"wam/UpperWristPitchJoint",
									"wam/LowerWristYawJoint"
								};
		std::vector < std::string > wam_joints(wam_jnts, wam_jnts + 7);
		wam_joint_state.name = wam_joints;
		wam_joint_state.name.resize(DOF);
		wam_joint_state.position.resize(DOF);
		wam_joint_state.velocity.resize(DOF);
		wam_joint_state.effort.resize(DOF);

        joint_pos_sub = n_.subscribe(remoteHost, 1, &MasterMasterROS::joint_pos_cb, this);
		wam_joint_state_pub = n_.advertise<sensor_msgs::JointState>("wam/joint_states_link", 1);
        //

		///* Create socket */
		//sock = socket(PF_INET, SOCK_DGRAM, 0);
		//if (sock == -1)
		//{
		//	syslog(LOG_ERR,"%s: Could not create socket.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
//
		///* Set socket to non-blocking, set flag associated with open file */
		//flags = fcntl(sock, F_GETFL, 0);
		//if (flags < 0)
		//{
		//	syslog(LOG_ERR,"%s: Could not get socket flags.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
		//flags |= O_NONBLOCK;
		//err = fcntl(sock, F_SETFL, flags);
		//if (err < 0)
		//{
		//	syslog(LOG_ERR,"%s: Could not set socket flags.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
//
		///* Maybe set UDP buffer size? */
		//buflenlen = sizeof(buflen);
		//err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		//if (err)
		//{
		//	syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
		//syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);
//
		//buflenlen = sizeof(buflen);
		//buflen = 5 * SIZE_OF_MSG;
		//err = setsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, buflenlen);
		//if (err)
		//{
		//	syslog(LOG_ERR,"%s: Could not set output buffer size.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
//
		//buflenlen = sizeof(buflen);
		//err = getsockopt(sock, SOL_SOCKET, SO_SNDBUF, (char *)&buflen, &buflenlen);
		//if (err)
		//{
		//	syslog(LOG_ERR,"%s: Could not get output buffer size.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
		//syslog(LOG_ERR,"%s: Note, output buffer is %d bytes.",__func__,buflen);
//
		///* Set up the bind address */
		//bind_addr.sin_family = AF_INET;
		//bind_addr.sin_port = htons(port);
		//bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
		//err = bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
		//if (err == -1)
		//{
		//	syslog(LOG_ERR,"%s: Could not bind to socket on port %d.",__func__,port);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
//
		///* Set up the other guy's address */
		//their_addr.sin_family = AF_INET;
		//their_addr.sin_port = htons(port);
		//err = ! inet_pton(AF_INET, remoteHost, &their_addr.sin_addr);
		//if (err)
		//{
		//	syslog(LOG_ERR,"%s: Bad IP argument '%s'.",__func__,remoteHost);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}
//
		///* Call "connect" to set datagram destination */
		//err = connect(sock, (struct sockaddr *)&their_addr, sizeof(struct sockaddr));
		//if (err)
		//{
		//	syslog(LOG_ERR,"%s: Could not set datagram destination.",__func__);
		//	throw std::runtime_error("(MasterMaster::MasterMaster): Ctor failed. Check /var/log/syslog.");
		//}


		if (em != NULL) {
			em->startManaging(*this);
		}
	}

	virtual ~MasterMasterROS() {
		this->mandatoryCleanUp();

		//close(sock);
	}

	bool isLinked() const { return linked; }
	void tryLink() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		ROS_INFO("numMissed: %i", numMissed);
		if (numMissed < NUM_MISSED_LIMIT) {
			linked = true;
		}
        //linked = true;
	}
	void unlink() { linked = false; }

protected:
	static const int SIZE_OF_MSG = DOF*sizeof(double);
	static const int NUM_MISSED_LIMIT = 500;
	int num_received;
	virtual void operate() {

		{
			// send() and recv() cause switches to secondary mode. The socket is
			// non-blocking, so this *probably* won't impact the control-loop
			// timing that much...
			barrett::thread::DisableSecondaryModeWarning dsmw;

			//send(sock, this->JpInput.getValue().data(), SIZE_OF_MSG, 0);

			//send(sock, this->input.getValue().data(), SIZE_OF_MSG, 0);

			if (numMissed < NUM_MISSED_LIMIT) {  // prevent numMissed from wrapping
				++numMissed;
			}

			for (size_t i = 0; i < DOF; i++) {
				wam_joint_state.position[i] = this->JpInput.getValue()[i];
			}
			wam_joint_state.header.stamp = ros::Time::now();
			wam_joint_state_pub.publish(wam_joint_state);
			ros::spinOnce();
			//while (true) {
			//  num_received=recv(sock, theirJp.data(), SIZE_OF_MSG, 0);
			//  if(num_received==SIZE_OF_MSG)
			//    {
			//	numMissed = 0;
			//    }
			//  else if(num_received==4*sizeof(double))
			//    {
			//      numMissed = 0;

			//      if(this->JpHandTool.getValue()[0]==1.0)
			//	{
			//	  theirJp[4]=this->JpInput.getValue()[4];
			//	  theirJp[5]=this->JpInput.getValue()[5];
			//	  theirJp[6]=this->JpInput.getValue()[6];
			//
			//	}
			//      else
			//	{
			//	  theirJp[4]=this->JpHandTool.getValue()[4];
			//	  theirJp[5]=this->JpHandTool.getValue()[5];
			//	  theirJp[6]=this->JpHandTool.getValue()[6];
			//	}
			//      /*
			//      theirJp[4]=this->input.getValue()[4];
			//      theirJp[5]=this->input.getValue()[5];
			//      theirJp[6]=this->input.getValue()[6];
			//      */
			//    }
			//  else
			//    {
			//      break;
			//    }
			//
			//}
		}

		if ( !linked  ||  numMissed >= NUM_MISSED_LIMIT) {
			linked = false;
			theirJp = this->JpInput.getValue();

			//theirJp = this->input.getValue();
		}


		jpOutputValue->setData(&theirJp);
		//this->outputValue->setData(&theirJp);
	}

    int sock;
	bool linked;
	int numMissed;
	jp_type theirJp;

private:
	DISALLOW_COPY_AND_ASSIGN(MasterMasterROS);
};


#endif /* ARM_LINKING_ROS_H_ */
