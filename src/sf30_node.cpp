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
#include <fcntl.h>    
#include <sensor_msgs/LaserScan.h>

int fdes;
bool exit_;

void SigintHandler(int sig)
{
  exit_=true;
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

bool isValidNumber(char a){
  
  if ((a>=48) && (a<=57))
    return true;
  else 
    return false;
  
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "sf30_node");
  ros::NodeHandle n;
  
  signal(SIGINT, SigintHandler);
  
  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("range",1);
  
  ros::Rate loop_rate(50);
 
  std::string s;
  n.param<std::string>("sf30_node/portname", s, "/dev/ttyUSB0");
  const char * portname = s.c_str();
  fdes = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fdes < 0)
  {
        ROS_ERROR("Error %d opening %s: %s", errno, portname, strerror (errno));
        return 1;
  } 
  set_interface_attribs (fdes, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
  set_blocking (fdes, 1);                    // set blocking

  char buf[1];
  int nc, i;
  float num=0.0;
  int confidence=0;
  tcflush( fdes, TCIFLUSH );
  ros::Time last_time=ros::Time::now();
  exit_=false;
  
  // Main loop
  while (ros::ok() && !exit_)
     {
       
      tcflush( fdes, TCIFLUSH ); // to get the most recent data, flush the buffer
      
      while(ros::ok() && !exit_){ // format of the data 00.00 m\n\n
	  
           nc = read (fdes, buf, 1);
	   
	   if ((nc==1) && (buf[0]=='m')){ // Look for the 'm'
	     
		num=0.0;
		confidence=0;
	     
		nc = read (fdes, buf, 1); // should be a Linebreak
		if ((nc!=1) || buf[0]!=10)  break;
		nc = read (fdes, buf, 1); // should be another Linebreak
		if ((nc!=1) || buf[0]!=10)  break;
		
		i=1; 
		do{ // Process the number before the point. 
		   nc = read (fdes, buf, 1); 
		   if ((nc==1) && (isValidNumber(buf[0]))){
		     num=num*i+atof(buf);
		     i=i*10;
		   }
		   else 
		     break;
		}
		while(1);
		
		if ((nc!=1) || (buf[0]!='.')){ // In normal situation, only exit the loop if find the decimal point
		  num=0.0;
		  break;
		}				
		
		nc = read (fdes, buf, 1); // should be the first decimal digit
		if ((nc!=1) || (!isValidNumber(buf[0]))){
		  num=0.0;
		  break;
		}	
		num=num+atof(buf)/10.0;
		
		nc = read (fdes, buf, 1); // should be the second decimal digit
		if ((nc!=1) || (!isValidNumber(buf[0]))){
		  num=0.0;
		  break;
		}	
		num=num+atof(buf)/100.0;
		
		confidence=1; // If we got so far, we can thrust the number in num!
		
		break;
	   } //if
	   
      } // while
             
      ros::Time now = ros::Time::now();
      ros::Duration duration=now-last_time;
      last_time=now;
      
      sensor_msgs::LaserScan data;
      data.header.frame_id = "sf30";
      data.header.stamp = ros::Time::now();
      data.scan_time =  duration.toSec();
      data.range_max=100.0;
      data.ranges.push_back(num);
      data.intensities.push_back((double)confidence); // Confidence = 0, range = 0 cannot be thrusted
      
      laser_pub.publish(data);
      
      ros::spinOnce();
      loop_rate.sleep();  
      
     }
}
