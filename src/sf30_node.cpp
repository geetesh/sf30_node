/*
* Copyright (c) 2016 Carnegie Mellon University, Guilherme Pereira <gpereira@ufmg.br>
*
* For License information please see the LICENSE file in the root directory.
*
*/

// S30 Laser rangefinder node- Guilherme Pereira, May, 2016
// You must set the sensor using the windows program first. The bound rate must be 115200 and the frequency larger than 50Hz


#include <ros/ros.h>
#include <signal.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>    /* For O_RDWR */
#include <sensor_msgs/LaserScan.h>

int fdes;

void SigintHandler(int sig)
{
  close(fdes);
  ros::Duration(0.5).sleep();
  ros::shutdown();
}

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                ROS_ERROR("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                ROS_ERROR("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                ROS_ERROR("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                ROS_ERROR("error %d setting term attributes", errno);
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "sf30_node");
  ros::NodeHandle n;
  
  signal(SIGINT, SigintHandler);
  
  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("/sf30/range",1);
  
  ros::Rate loop_rate(50);
 
  std::string s;
  n.param<std::string>("sf30_node/portname", s, "/dev/ttyUSB0");
  const char * portname = s.c_str();
  fdes = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fdes < 0)
  {
        ROS_ERROR("error %d opening %s: %s", errno, portname, strerror (errno));
        return 1;
  } 
  set_interface_attribs (fdes, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
  set_blocking (fdes, 0);                    // set no blocking

  char buf[1];
  int nc, i;
  float num=0.0;
  int confidence=0;
  nc = read (fdes, buf, 10);
  ros::Time last_time=ros::Time::now();
 
  // Main loop
  while (ros::ok())
     {
      while(ros::ok){       
           nc = read (fdes, buf, 1);
	   if (buf[0]=='\n'){
	        nc = read (fdes, buf, 1); // Linebreak
		i=1;
		num=0.0;
		do{
		   nc = read (fdes, buf, 1); 
		   if (buf[0]=='-') // invalid data
		     break;
		   if (buf[0]!='.')
		     num=num*i+atof(buf);
		   i=i*10;
		}
		while (buf[0]!='.');
		
		if (buf[0]=='-'){
		  num=0.0;
		  confidence=0;
		  break;
		}
		
		confidence=1;
		
		nc = read (fdes, buf, 1); // first decimal digit
		num=num+atof(buf)/10.0;
		
		nc = read (fdes, buf, 1); // second decimal digit
		num=num+atof(buf)/100.0;
		
		tcflush( fdes, TCIFLUSH ); 
		
		break;
	   }
      }  
      
         
      ros::Time now = ros::Time::now();
      ros::Duration duration=now-last_time;
      last_time=now;
      
      sensor_msgs::LaserScan data;
      data.header.frame_id = "sf30";
      data.header.stamp = ros::Time::now();
      data.scan_time =  duration.toSec();
      data.range_max=100.0;
      data.ranges.push_back(num);
      data.intensities.push_back((double)confidence);
      
      laser_pub.publish(data);
      
      ros::spinOnce();
      loop_rate.sleep();  
      
     }
}
