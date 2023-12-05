#include <stdlib.h>
#include <termios.h>

	// function to get keyboard input from a user without waiting for an input to occur
	// function provided by ChatGPT https://chat.openai.com/share/c5e0891f-7488-41d0-9792-0faa99b4d32b
char nonBlockingGetch() {
	struct termios oldt, newt;
	char ch = 0;
	int oldf = fcntl(STDIN_FILENO, F_GETFL);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	int ret = read(STDIN_FILENO, &ch, 1);

	if (ret < 0) {
		ch = 0;
	}

	fcntl(STDIN_FILENO, F_SETFL, oldf);
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

int main(int argc, char **argv) {   

	char option = ' ';
	while (ros::ok()){
		
		option = nonBlockingGetch();

		ROS_INFO("I am running\n");

		if(option == 'h'){      	// drive forward
			ROS_INFO("\n\n\n\nEnabling Manual Drive Mode\n\n\n\n");
			system("roslaunch project-3 keyboard_teleop.launch");
			system("clear");
			ROS_INFO("\nDisabled Manual Drive Mode\n");
		} 

		else if (option == 'q'){	//greetings
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/hello.wav");
		}

		else if (option == 'w'){	//wilhelm
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/wilhelm.wav");
		}

		else if (option == 'e'){	//move out the way
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/move.wav");
		}

		else if (option == 'r'){	//dreamer
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/dreamer.wav");
		}

		else if (option == 't'){	//omar
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/omar.wav");
		}

		else if (option == 'y'){	//love
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/love.wav");
		}

		else if (option == 'u'){	//thanks
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/thanks.wav");
		}
		
		else if (option == 'a'){	//goToDreamer
			system("python /home/lumr0000/catkin_ws/src/project-3/scripts/goToDreamer.py");
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/dreamer.wav");
		}

		else if (option == 's'){	//goToOmar
			system("python /home/lumr0000/catkin_ws/src/project-3/scripts/goToOmar.py");
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/omar.wav");
		}
		
		else if (option == 'd'){	//goToLove
			system("python /home/lumr0000/catkin_ws/src/project-3/scripts/goToLove.py");
			system("rosrun sound_play play.py /home/lumr0000/catkin_ws/src/audio_common/sound_play/sounds/love.wav");
		}
	}
	return 0;
}
