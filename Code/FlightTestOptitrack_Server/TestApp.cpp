// STL and winsock
#define _USE_MATH_DEFINES
#include <cstring>
#include <cmath>
#include <iostream>
#include <thread>
#include <mutex>
#include <semaphore>
#include <chrono>
#include <ratio>
#include <cstdio>
#include <algorithm>

// NatNet SDK includes
#include "./include/NatNetTypes.h"
#include "./include/NatNetCAPI.h"
#include "./include/NatNetClient.h"

// XInput SDK includes
#include <Windows.h>
#include <Xinput.h>

using std::string;
using std::cout;
using std::endl;
using namespace std::this_thread;
using namespace std::chrono;

// local and server IP for optitrack, set to 127.0.0.1 when using loopback
constexpr char Local_IP[] = "127.0.0.1";
constexpr char Server_IP[] = "127.0.0.1";

// frame rate settings
constexpr uint32_t Frame_Rate = 360;
constexpr double Frame_Time = 1000000.0F / Frame_Rate;

// callback function to be called when new Optitrack data comes in
void NATNET_CALLCONV OptitrackDataHandler(sFrameOfMocapData* data, void* pUserData);
// a task (thread) that reads the controller value and send to the drone.
void ControllerTask();
// a task (thread) that reads message from the receiver. 
void ReceiveMessageTask();

// some other global defs
NatNetClient* OptitrackClient = nullptr;
sNatNetClientConnectParams g_connectParams;
sServerDescription g_serverDescription;
sDataDescriptions* g_pDataDefs = nullptr;

// struct specs
// make sure you properly align the memory
// i.e. 8 bytes at a time

// command for motor
union Command
{
	struct
	{
		int64_t zero_time;
		int32_t period;
		uint32_t t_offset; // how long after zero_time to start switching from throttle 0 -> 1
		uint32_t throttle_0;
		uint32_t throttle_1;
	};
	uint8_t raw[24];
};

union Motor_FB_t
{
	struct
	{
		int64_t time;
		uint32_t throttle;
		uint32_t eRPM;
	};
	uint8_t raw[16];
};

union AngleMsg
{
	struct
	{
		int64_t zero_time;
		int32_t period;
		uint32_t throttle;
	};
	uint8_t raw[16];
};

// set single throttle
int set_throttle_val;

// final target center point
float target_point_final[3] = { -0.2F, 0.0F, 1.4F };
// target center point
float target_point[3] = { -0.2F, 0.0F, 1.4F };

// buffer dir,x,y,z for filtering
constexpr uint32_t buf_len = 100;
float dir_buf[buf_len] = { 0 };
float x_buf[buf_len] = { 0 };
float y_buf[buf_len] = { 0 };
float z_buf[buf_len] = { 0 };
// integral of x,y,z
float x_int = 0;
float y_int = 0;
float z_int = 0;
float xy_int_max = 6.0F;
float z_int_max = 3.0F;
// pointer to newest content
// to get the oldest content, use (buf_ptr+1)%buf_len
uint32_t buf_ptr = buf_len - 1;
// latest frame number
int32_t latest_nframe = 0;
// access control for Optitrack data
std::mutex buf_mutex;

// kernels for derivatives
// note that kernels are directly tied to Frame_Rate and buf_len
constexpr float lpf_filt_ker[buf_len] = { 0.04190F, 0.04749F, 0.05269F, 0.05731F, 0.06122F, 0.06426F, 0.06634F, 0.06739F, 0.06736F, 0.06627F, 0.06416F, 0.06111F, 0.05723F, 0.05265F, 0.04754F, 0.04205F, 0.03636F, 0.03064F, 0.02502F, 0.01966F, 0.01466F, 0.01011F, 0.00608F, 0.00261F, -0.00029F, -0.00263F, -0.00443F, -0.00573F, -0.00658F, -0.00704F, -0.00716F, -0.00700F, -0.00662F, -0.00607F, -0.00540F, -0.00466F, -0.00386F, -0.00306F, -0.00227F, -0.00152F, -0.00084F, -0.00022F, 0.00030F, 0.00073F, 0.00105F, 0.00127F, 0.00138F, 0.00141F, 0.00135F, 0.00122F, 0.00103F, 0.00082F, 0.00059F, 0.00036F, 0.00016F, -0.00001F, -0.00014F, -0.00022F, -0.00025F, -0.00024F, -0.00019F, -0.00012F, -0.00005F, 0.00003F, 0.00009F, 0.00013F, 0.00013F, 0.00010F, 0.00004F, -0.00005F, -0.00015F, -0.00026F, -0.00036F, -0.00044F, -0.00048F, -0.00049F, -0.00045F, -0.00037F, -0.00025F, -0.00010F, 0.00007F, 0.00025F, 0.00042F, 0.00056F, 0.00068F, 0.00075F, 0.00077F, 0.00074F, 0.00067F, 0.00056F, 0.00043F, 0.00029F, 0.00015F, 0.00002F, -0.00007F, -0.00014F, -0.00016F, -0.00014F, -0.00009F, 0.00000F };
constexpr float vel_filt_ker[buf_len] = { 3.25190F, 1.93669F, 1.31781F, 1.12792F, 0.94081F, 0.83492F, 0.72085F, 0.63830F, 0.54845F, 0.47471F, 0.39537F, 0.32587F, 0.25238F, 0.18589F, 0.11685F, 0.05354F, -0.01100F, -0.07033F, -0.12972F, -0.18401F, -0.23734F, -0.28550F, -0.33184F, -0.37287F, -0.41141F, -0.44454F, -0.47469F, -0.49943F, -0.52088F, -0.53702F, -0.54977F, -0.55745F, -0.56181F, -0.56146F, -0.55803F, -0.55036F, -0.53999F, -0.52595F, -0.50969F, -0.49040F, -0.46943F, -0.44611F, -0.42169F, -0.39560F, -0.36899F, -0.34136F, -0.31376F, -0.28573F, -0.25822F, -0.23080F, -0.20432F, -0.17836F, -0.15366F, -0.12980F, -0.10743F, -0.08613F, -0.06646F, -0.04799F, -0.03120F, -0.01566F, -0.00177F, 0.01090F, 0.02200F, 0.03195F, 0.04045F, 0.04792F, 0.05409F, 0.05935F, 0.06348F, 0.06684F, 0.06923F, 0.07097F, 0.07188F, 0.07226F, 0.07193F, 0.07117F, 0.06982F, 0.06810F, 0.06589F, 0.06335F, 0.06040F, 0.05717F, 0.05358F, 0.04974F, 0.04561F, 0.04122F, 0.03663F, 0.03176F, 0.02675F, 0.02142F, 0.01605F, 0.01019F, 0.00438F, -0.00233F, -0.00893F, -0.01790F, -0.02712F, -0.04928F, -0.09321F, 0.00000F };
constexpr float acc_filt_ker[buf_len] = { 36.22320F, 24.48570F, 11.51790F, 8.26903F, 5.01190F, 3.59574F, 2.03367F, 1.25337F, 0.35496F, -0.11736F, -0.68370F, -0.98652F, -1.36640F, -1.57010F, -1.83858F, -1.98278F, -2.18251F, -2.29109F, -2.44816F, -2.53671F, -2.66968F, -2.75146F, -2.86889F, -2.94110F, -3.04457F, -3.11147F, -3.20681F, -3.27248F, -3.36461F, -3.43284F, -3.52635F, -3.60107F, -3.70066F, -3.78624F, -3.89709F, -3.99856F, -4.12671F, -4.24976F, -4.40314F, -4.54051F, -4.58314F, -4.52444F, -4.43718F, -4.31022F, -4.17737F, -4.02049F, -3.86579F, -3.69421F, -3.52830F, -3.34923F, -3.17727F, -2.99416F, -2.81853F, -2.63272F, -2.45408F, -2.26539F, -2.08289F, -1.88923F, -1.69899F, -1.49041F, -1.27762F, -1.06293F, -0.86379F, -0.66395F, -0.47788F, -0.28994F, -0.11453F, 0.06384F, 0.23058F, 0.40129F, 0.56096F, 0.72552F, 0.87934F, 1.03894F, 1.18773F, 1.34318F, 1.48718F, 1.63902F, 1.77425F, 1.88673F, 1.96065F, 2.03156F, 2.07836F, 2.13109F, 2.16199F, 2.20539F, 2.22635F, 2.26759F, 2.28374F, 2.33194F, 2.34997F, 2.42134F, 2.45355F, 2.58607F, 2.66363F, 2.98534F, 3.24067F, 4.85816F, 6.04118F, 0.00000F };

// kernel lags in us
constexpr float val_filt_lag = 9.0F * Frame_Time;
constexpr float vel_filt_lag = 18.0F * Frame_Time;
constexpr float acc_filt_lag = 40.0F * Frame_Time;

// get local time in us
int64_t get_time()
{
	return static_cast<int64_t>(high_resolution_clock().now().time_since_epoch() / microseconds(1));
}

// dead lock sleep for us using get_time()
void sleep_us(int64_t dur)
{
	int64_t start_time = get_time();
	while (get_time() - start_time < dur) {}
}

// apply filter to data, no precautions for concurrency
float apply_filter(const float* const buf, const uint32_t buf_ptr, const float* const ker)
{
	float val = 0;

	for (int i = 0; i < buf_len; i++)
	{
		val += buf[(buf_ptr + 1 + i) % buf_len] * ker[buf_len - 1 - i];
	}

	return val;
}

HANDLE hSerial;

FILE* data_log;

int main(int argc, char* argv[])
{
	// Get current time
	time_t now_c = system_clock::to_time_t(system_clock::now());
	struct tm timeinfo;
	localtime_s(&timeinfo, &now_c);

	char log_filename[100] = "";
	sprintf_s(log_filename, sizeof(log_filename), "%02d%02d%02d_%02d_%02d.csv", (timeinfo.tm_year) % 100, 1 + timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min);

	fopen_s(&data_log, log_filename, "w");
	fprintf(data_log, "DATA  ,  time,rot_freq,  throttle_0,throttle_1,sw_angle  ,  integral_x,y,z  ,  filt_dir,x,y,z  ,  vel_dir,x,y,z  ,  acc_dir,x,y,z\n");
	fprintf(data_log, "FB  ,  time,throttle,eRPM\n");
	fprintf(data_log, "RAW  ,  time  ,  x,y,z  ,  qx,qy,qz,qw \n\n");

	// The first part is about connecting to optitrack
	cout << "------ Initializing Optitrack ------\n";

	// connect to NatNet
	ErrorCode ret = ErrorCode_OK;

	// Create a NatNet client
	OptitrackClient = new NatNetClient();

	// Set the Client's frame callback handler
	ret = OptitrackClient->SetFrameReceivedCallback(OptitrackDataHandler, OptitrackClient);

	// Specify client PC's IP address, Motive PC's IP address, and network connection type
	g_connectParams.localAddress = Local_IP;
	g_connectParams.serverAddress = Server_IP;
	g_connectParams.connectionType = ConnectionType_Multicast;

	// Connect to Motive
	ret = OptitrackClient->Connect(g_connectParams);
	if (ret != ErrorCode_OK)
	{
		cout << "Error at connection: " << ret;
		return 1;
	}
	else
	{
		cout << "Optitrack connected!\n";
	}

	// set frame rate 
	void* response = nullptr;
	int nBytes = 0;

	// check if frame rate is consistent with our settings
	ret = OptitrackClient->SendMessageAndWait("FrameRate", &response, &nBytes);

	if (ret == ErrorCode_OK)
	{
		float fRate = *((float*)response);
		if (static_cast<int>(fRate) == Frame_Rate)
		{
			printf("Frame rate is : %3.2f .\n", fRate);
		}
		else
		{
			printf("Current frame rate of %3.2f is inconsistent with expected frame rate of %d .\n", fRate, Frame_Rate);
			return 1;
		}
	}
	else
	{
		printf("Error reading frame rate!\n");
		return 1;
	}

	cout << "------ Optitrack initialization successful ------\n";

	cout << "------ Open serial port ------\n";

	// Open the serial port
	hSerial = CreateFile(L"\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	if (hSerial == INVALID_HANDLE_VALUE)
	{
		std::cerr << "Error opening serial port" << std::endl;
		return 1;
	}

	// Set serial port parameters
	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams))
	{
		std::cerr << "Error getting serial port state" << std::endl;
		CloseHandle(hSerial);
		return 1;
	}
	dcbSerialParams.BaudRate = 2000000;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams))
	{
		std::cerr << "Error setting serial port state" << std::endl;
		CloseHandle(hSerial);
		return 1;
	}

	// Set timeouts
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = MAXDWORD;  // Disable interval timeout
	timeouts.ReadTotalTimeoutConstant = 0;    // No total timeout
	timeouts.ReadTotalTimeoutMultiplier = 0;  // No total timeout
	timeouts.WriteTotalTimeoutConstant = 0;   // No total timeout
	timeouts.WriteTotalTimeoutMultiplier = 0; // No total timeout
	if (!SetCommTimeouts(hSerial, &timeouts))
	{
		std::cerr << "Error setting serial port timeouts" << std::endl;
		CloseHandle(hSerial);
		return 1;
	}

	cout << "------ Serial port active ------\n";

	XINPUT_STATE state;
	DWORD result = XInputGetState(0, &state);

	if (result != ERROR_SUCCESS)
	{
		std::cerr << "Error connecting to joystick" << std::endl;
		return 1;
	}

	// Create threads for reading and writing
	std::thread(ReceiveMessageTask).detach();

	// Start the thread for sending messages from controller
	std::thread(ControllerTask).detach();

	// clear input
	printf("\033[2J");
	printf("\033[1;0f------ Local info ------");
	printf("\033[2;0f\033[K{ Frame time ,  freq  ,{ th_0 , th_1 , sw_ang},{   dir  ,    x   ,    y   ,    z   }}");
	printf("\033[5;0f------ Feedback info ------");
	printf("\033[6;0f\033[K{ Speed time , th_v ,  eRPM }");
	printf("\033[7;0f\033[K{ %10lld , %4d , %5d }", 0LL, 0, 0);
	printf("\033[12;0f------ Launcher info ------");

	// wait on main thread
	while (true) {}

	// file cleanup
	fclose(data_log);

	// Clean up
	if (OptitrackClient)
	{
		OptitrackClient->Disconnect();
		delete OptitrackClient;
	}

	if (g_pDataDefs)
	{
		NatNet_FreeDescriptions(g_pDataDefs);
		g_pDataDefs = NULL;
	}

	// Close the serial port
	CloseHandle(hSerial);

	return 0;
}

// convert angle into [-pi,pi) range
float angle_convert(float a)
{
	return a - floor((a + M_PI) / (2.0 * M_PI)) * (2.0 * M_PI);
}

void NATNET_CALLCONV OptitrackDataHandler(sFrameOfMocapData* data, void* pUserData)
{
	// update buffers
	buf_mutex.lock();

	// reset latest frame number when just start recording
	if (data->iFrame <= 10)
	{
		latest_nframe = data->iFrame - 1;
	}

	auto temp_rb = data->RigidBodies[0];
	int32_t nframe = data->iFrame;

	// printf("\033[4;0f\033[K{%lld,%f}\n", int64_t(Frame_Time * nframe), (1.0F - 2.0F * temp_rb.qx * temp_rb.qx - 2.0F * temp_rb.qy * temp_rb.qy));

	// quit if the rigid body is not tracked in this frame
	// or direction tracking is likely failed
	if (!(temp_rb.params & 0x01) || (1.0F - 2.0F * temp_rb.qx * temp_rb.qx - 2.0F * temp_rb.qy * temp_rb.qy) > -0.9F)
	{
		buf_mutex.unlock();
		return;
	}

	float dir = atan2f(temp_rb.qx * temp_rb.qy + temp_rb.qz * temp_rb.qw, 0.5F - temp_rb.qy * temp_rb.qy - temp_rb.qz * temp_rb.qz);

	if (latest_nframe == 0 || nframe - latest_nframe == 1)
	{
		x_int += ((temp_rb.x + x_buf[buf_ptr]) / 2.0F - target_point[0]) * Frame_Time / 1000000.0F;
		y_int += ((temp_rb.y + y_buf[buf_ptr]) / 2.0F - target_point[1]) * Frame_Time / 1000000.0F;
		z_int += ((temp_rb.z + z_buf[buf_ptr]) / 2.0F - target_point[2]) * Frame_Time / 1000000.0F;

		buf_ptr = (buf_ptr + 1) % buf_len;
		x_buf[buf_ptr] = temp_rb.x;
		y_buf[buf_ptr] = temp_rb.y;
		z_buf[buf_ptr] = temp_rb.z;
		dir_buf[buf_ptr] = dir;
		latest_nframe = nframe;
	}
	else if (nframe > latest_nframe)
	{
		// deal with lost frames
		// give up if more than half a rotation is lost
		constexpr int32_t max_tolerable_lost_frames = 17;
		if (nframe - latest_nframe > max_tolerable_lost_frames)
		{
			// when give up, we clear the buffer
			for (int i = 0; i < buf_len; i++)
			{
				x_buf[i] = 0;
				y_buf[i] = 0;
				z_buf[i] = 0;
				dir_buf[i] = 0;
			}
			buf_ptr = 0;
			x_buf[buf_ptr] = temp_rb.x;
			y_buf[buf_ptr] = temp_rb.y;
			z_buf[buf_ptr] = temp_rb.z;
			dir_buf[buf_ptr] = dir;

			x_int = 0;
			y_int = 0;
			z_int = 0;

			printf("\033[4;0f\033[KReset @ time %lld because too many frames (%lld) are lost!\n", int64_t(Frame_Time * nframe), nframe - latest_nframe);
		}
		else
		{
			// integral terms
			x_int += ((temp_rb.x + x_buf[buf_ptr]) / 2.0F - target_point[0]) * (nframe - latest_nframe) * Frame_Time / 1000000.0F;
			y_int += ((temp_rb.y + y_buf[buf_ptr]) / 2.0F - target_point[1]) * (nframe - latest_nframe) * Frame_Time / 1000000.0F;
			z_int += ((temp_rb.z + z_buf[buf_ptr]) / 2.0F - target_point[2]) * (nframe - latest_nframe) * Frame_Time / 1000000.0F;

			// else we simply do linear interpolation
			float x_diff = (temp_rb.x - x_buf[buf_ptr]) / float(nframe - latest_nframe);
			float y_diff = (temp_rb.y - y_buf[buf_ptr]) / float(nframe - latest_nframe);
			float z_diff = (temp_rb.z - z_buf[buf_ptr]) / float(nframe - latest_nframe);
			float dir_diff = angle_convert(dir - dir_buf[buf_ptr]) / float(nframe - latest_nframe);

			for (int i = 1; i < nframe - latest_nframe + 1; i++)
			{
				auto temp_ptr = (buf_ptr + i) % buf_len;
				x_buf[temp_ptr] = x_buf[buf_ptr] + x_diff * i;
				y_buf[temp_ptr] = y_buf[buf_ptr] + y_diff * i;
				z_buf[temp_ptr] = z_buf[buf_ptr] + z_diff * i;
				dir_buf[temp_ptr] = angle_convert(dir_buf[buf_ptr] + dir_diff * i);
			}
			buf_ptr = (buf_ptr + nframe - latest_nframe) % buf_len;
		}

		latest_nframe = nframe;
	}

	// clamp integral terms
	x_int = std::clamp(x_int, -xy_int_max, xy_int_max);
	y_int = std::clamp(y_int, -xy_int_max, xy_int_max);
	z_int = std::clamp(z_int, -z_int_max, z_int_max);

	buf_mutex.unlock();

	// print to file when recording
	if (data->params & 0x01)
	{
		fprintf(data_log, "RAW  ,  %10lld  ,  %4.4f,%4.4f,%4.4f  ,  %4.4f,%4.4f,%4.4f,%4.4f\n", int64_t(nframe * Frame_Time), temp_rb.x, temp_rb.y, temp_rb.z, temp_rb.qx, temp_rb.qy, temp_rb.qz, temp_rb.qw);
	}

	return;
}

void ControllerTask()
{
	// absolute max and min throttle
	constexpr int32_t max_throttle = 1999, min_throttle = 300;

	// controller map max and min throttle
	constexpr float max_ctrl_throttle = 1800, min_ctrl_throttle = 700;

	// some temporary buffers
	float temp_dir_buf[buf_len] = { 0 };
	float temp_x_buf[buf_len] = { 0 };
	float temp_y_buf[buf_len] = { 0 };
	float temp_z_buf[buf_len] = { 0 };
	// pointer to newest content
	// to get the oldest content, use (buf_ptr+1)%buf_len
	uint32_t temp_buf_ptr = buf_len - 1;
	// latest frame number
	int32_t temp_latest_nframe = 0;

	bool XYcontrol_on = false;
	uint32_t ref_throttle = 0;

	while (true)
	{
		sleep_for(milliseconds(5));

		XINPUT_STATE state;
		DWORD result = XInputGetState(0, &state);

		if (result == ERROR_SUCCESS && state.dwPacketNumber)
		{
			// printf("\033[18;0f\033[K %d", state.Gamepad.wButtons);

			// control average throttle using angle of right joystick
			float padvalx = state.Gamepad.sThumbRX, padvaly = state.Gamepad.sThumbRY;
			// left pad status
			int padval = state.Gamepad.wButtons & 0xF;
			int padx = 0, pady = 0;
			switch (padval)
			{
			case 1:
				pady = 1;
				break;
			case 2:
				pady = -1;
				break;
			case 4:
				padx = -1;
				break;
			case 8:
				padx = 1;
				break;
			}
			
			// whether XY control is enabled
			XYcontrol_on = ((state.Gamepad.wButtons & 0x200) != 0);

			// launch or land
			int launchval = state.Gamepad.wButtons & 0x4000;
			int landval = state.Gamepad.wButtons & 0x8000;

			// 0 for not started
			// 1 for started but not speed up enough
			// 2 for speed up and about to launch
			static int drone_flight_state = 0;
			// last time launch is triggered
			static int64_t last_start_launch_time = 0;

			constexpr int64_t start_drone_motor_delay = 2400000;

			if (launchval)
			{
				if (drone_flight_state == 0)
				{
					// launch
					// send message only to launcher
					uint8_t cmd_array[5] = { 1,0,0,0,0 };
					constexpr int32_t launch_velocity = int32_t(18.0 * 36000.0); // launch angular velocity, unit in 0.01deg
					memcpy(cmd_array + 1, &launch_velocity, sizeof(int32_t));
					DWORD bytesWritten;
					WriteFile(hSerial, cmd_array, 5, &bytesWritten, NULL);
					// start timing and start drone motor after start_launch_delay
					drone_flight_state = 1;
					last_start_launch_time = get_time();
				}
			}
			if (landval)
			{
				// cancel launch
				// send message only to launcher
				uint8_t cmd_array[2] = { 1,23 };
				DWORD bytesWritten;
				WriteFile(hSerial, cmd_array, 2, &bytesWritten, NULL);

				drone_flight_state = 0;
				last_start_launch_time = 0;
			}


			// reset integral term before launch
			if (drone_flight_state == 1)
			{
				buf_mutex.lock();
				x_int = 0;
				y_int = 0;
				z_int = 0;
				buf_mutex.unlock();

				// delayed launch
				if (get_time() - last_start_launch_time > start_drone_motor_delay)
				{
					drone_flight_state = 2;
				}
			}

			// print joystick button status for debug
			// printf("\033[9;0f\033[K%4x", state.Gamepad.wButtons);

			int32_t avg_throttle = 0, throttle_diff = 0;
			int64_t throttle_time = 0;

			//// average throttle control using joystick input (inactive for now)
			//float norm = sqrtf(padvalx * padvalx + padvaly * padvaly);
			//float ang = atan2f(padvalx, padvaly) / M_PI;
			//ang += (ang < 0) ? 2.0F : 0.0F;

			//if (norm > 20000 && 0.15 <= ang && ang <= 1.8)
			//{
			//	avg_throttle = uint32_t(((ang > 1.65) ? 1 : (ang - 0.15) / 1.5) * (max_ctrl_throttle - min_ctrl_throttle) + min_ctrl_throttle);
			//}

			// compute state of the drone
			// copy current buffer into temporary buffer so we can filter later
			buf_mutex.lock();
			memcpy(temp_dir_buf, dir_buf, sizeof(dir_buf));
			memcpy(temp_x_buf, x_buf, sizeof(x_buf));
			memcpy(temp_y_buf, y_buf, sizeof(y_buf));
			memcpy(temp_z_buf, z_buf, sizeof(z_buf));
			temp_buf_ptr = buf_ptr;
			temp_latest_nframe = latest_nframe;

			auto x_int_temp = x_int;
			auto y_int_temp = y_int;
			auto z_int_temp = z_int;
			buf_mutex.unlock();

			// convert dir buffer to continuous angle
			float last_val = temp_dir_buf[(temp_buf_ptr + 1) % buf_len];
			float accum = 0;
			for (int i = 1; i < buf_len; i++)
			{
				accum -= floor(((temp_dir_buf[(i + temp_buf_ptr + 1) % buf_len] - last_val) + M_PI) / (2.0 * M_PI)) * (2.0 * M_PI);
				last_val = temp_dir_buf[(i + temp_buf_ptr + 1) % buf_len];
				temp_dir_buf[(i + temp_buf_ptr + 1) % buf_len] += accum;
			}

			// apply filter to temporary buffer to obtain x, v, and a
			float filt_dir = angle_convert(apply_filter(temp_dir_buf, temp_buf_ptr, lpf_filt_ker));
			float filt_x = apply_filter(temp_x_buf, temp_buf_ptr, lpf_filt_ker);
			float filt_y = apply_filter(temp_y_buf, temp_buf_ptr, lpf_filt_ker);
			float filt_z = apply_filter(temp_z_buf, temp_buf_ptr, lpf_filt_ker);

			float filt_v_dir = apply_filter(temp_dir_buf, temp_buf_ptr, vel_filt_ker);
			float filt_v_x = apply_filter(temp_x_buf, temp_buf_ptr, vel_filt_ker);
			float filt_v_y = apply_filter(temp_y_buf, temp_buf_ptr, vel_filt_ker);
			float filt_v_z = apply_filter(temp_z_buf, temp_buf_ptr, vel_filt_ker);

			float filt_a_dir = apply_filter(temp_dir_buf, temp_buf_ptr, acc_filt_ker);
			float filt_a_x = apply_filter(temp_x_buf, temp_buf_ptr, acc_filt_ker);
			float filt_a_y = apply_filter(temp_y_buf, temp_buf_ptr, acc_filt_ker);
			float filt_a_z = apply_filter(temp_z_buf, temp_buf_ptr, acc_filt_ker);

			//// resets integration terms when filt_v_dir is smaller than 20Pi
			//if (fabs(filt_v_dir) < 20.0F * M_PI)
			//{
			//	buf_mutex.lock();
			//	x_int = 0;
			//	y_int = 0;
			//	z_int = 0;
			//	buf_mutex.unlock();
			//}

			// PID controller parameters for z and xy
			constexpr float kp_z = 500.0F, kd_z = 800.0F, ki_z = 250.0F;
			constexpr float kp_xy = 1268.2F, ki_xy = 0.0F, kd_xy = 1412.7F, ka_xy = 622.7F;
			constexpr float kp_yx = 1841.7F, ki_yx = 0.0F, kd_yx = 1744.3F, ka_yx = 871.7F;
			constexpr float x_max = 0.25F;

			// if launched, then use PID controller based on filt_z, filt_v_z to control throttle
			if (drone_flight_state == 2)
			{
				ref_throttle = 1200;

				constexpr float start_raise_target_delay = 1500000.0F;
				constexpr float relax_time = 500000.0F;
				constexpr float initial_ratio = 0.8F;
				
				buf_mutex.lock();
				target_point[2] = target_point_final[2] * (std::clamp(float(get_time() - last_start_launch_time - start_drone_motor_delay - start_raise_target_delay) / relax_time, 0.0F, 1.0F) * (1.0F - initial_ratio) + initial_ratio);
				buf_mutex.unlock();

				// PID controller for height control only
				avg_throttle = uint32_t(std::clamp(ref_throttle - kp_z * std::clamp(filt_z - target_point[2],-500.0F,500.0F) - ki_z * z_int_temp - kd_z * filt_v_z, min_ctrl_throttle, max_ctrl_throttle));
			}
			else
			{
				avg_throttle = 0;
			}

			//// target thrust vector
			//constexpr float k_p = 300;
			//float throttle_diff_x, throttle_diff_y;
			//throttle_diff_x = - k_p * (filt_x - target_point[0]);
			//throttle_diff_y = - k_p * (filt_y - target_point[1]);

			// compute target thrust vector using PID controller
			float throttle_diff_x, throttle_diff_y;
			throttle_diff_x = -kp_xy * std::clamp(filt_x - target_point[0], -x_max, x_max) - kp_yx * std::clamp(filt_y - target_point[1], -x_max, x_max) - kd_xy * filt_v_x - kd_yx * filt_v_y - ka_xy * filt_a_x - ka_yx * filt_a_y - ki_xy * x_int_temp;
			throttle_diff_y = -kp_xy * std::clamp(filt_y - target_point[1], -x_max, x_max) + kp_yx * std::clamp(filt_x - target_point[0], -x_max, x_max) - kd_xy * filt_v_y + kd_yx * filt_v_x - ka_xy * filt_a_y + ka_yx * filt_a_x - ki_yx * y_int_temp;

			// if xy control is enabled, disable x and y control
			if (!XYcontrol_on)
			{
				throttle_diff_x = 0;
				throttle_diff_y = 0;
			}
			
			if (padval != 0)
			{
				throttle_diff_x = 500 * padx;
				throttle_diff_y = 500 * pady;
			}

			// thrust vector amplitude
			throttle_diff = sqrt(throttle_diff_x * throttle_diff_x + throttle_diff_y * throttle_diff_y);
			// limit the amplitude to be less than 400
			throttle_diff = std::clamp(throttle_diff, 0, 500);

			Command cmd = { {0} };

			//// for positive rotation speed (this code needs to be corrected)
			//// period
			//cmd.period = 2000000.0F * M_PI / filt_v_dir;
			//// zero angle time
			//cmd.zero_time = temp_latest_nframe * Frame_Time - 1000000.0F * filt_dir / filt_v_dir - val_filt_lag;
			//// change thrust time offset
			//float sw_angle = (atan2(throttle_diff_y, throttle_diff_x)) / (2.0F * M_PI) + 0.5F;
			//// if sw_angle is larger than a half, then we need to set thrust to throttle 1 first, then 0.
			//bool invert = (sw_angle > 0.5F);
			//// set t_offset
			//cmd.t_offset = uint32_t((invert ? sw_angle - 0.5F : sw_angle) * cmd.period);

			// for negative rotation speed
			// period
			cmd.period = - 2000000.0F * M_PI / filt_v_dir;
			// zero angle time
			cmd.zero_time = temp_latest_nframe * Frame_Time - 1000000.0F * filt_dir / filt_v_dir - val_filt_lag;
			// the angle corresponding to low throttle start time.
			float sw_angle = atan2(throttle_diff_y, throttle_diff_x) / (2.0F * M_PI) - 0.25F; // -0.15F; // the last term is compensation
			// convert sw_angle to [-0.5,0.5) range
			sw_angle = (sw_angle < -0.5F) ? sw_angle + 1.0F : sw_angle;
			// if sw_angle is between (-0.5,0], then we need to set to low throttle first, then high.
			bool invert = (sw_angle < 0.0F);
			// set t_offset
			cmd.t_offset = uint32_t((invert ? -sw_angle : 0.5F - sw_angle) * cmd.period);

			// printf("\033[18;0f\033[K{ %4.4f, %4.4f, %4.4f, %d , %d }", sw_angle, (invert ? sw_angle - 0.5F : sw_angle), (invert ? -sw_angle : 0.5F - sw_angle), cmd.t_offset, cmd.period);

			if (avg_throttle == 0)
			{
				cmd.throttle_0 = 0;
				cmd.throttle_1 = 0;
			}
			else
			{
				if (avg_throttle > (max_throttle + min_throttle) / 2)
				{
					if (avg_throttle + throttle_diff > max_throttle)
					{
						throttle_diff = max_throttle - avg_throttle;
					}
				}
				else if (avg_throttle - throttle_diff < min_throttle)
				{
					throttle_diff = avg_throttle - min_throttle;
				}

				if (invert)
				{
					cmd.throttle_1 = avg_throttle + throttle_diff;
					cmd.throttle_0 = avg_throttle - throttle_diff;
				}
				else
				{
					cmd.throttle_0 = avg_throttle + throttle_diff;
					cmd.throttle_1 = avg_throttle - throttle_diff;
				}
			}

			//// const throttle for tests
			//cmd.throttle_0 = avg_throttle;
			//cmd.throttle_1 = avg_throttle;

			// move cursor to line 0, column 0
			// print {frame time, frequency, sw_angle (0~1 -> 0~360deg), throttle 0, throttle 1,{dir,x,y,z}}
			printf("\033[3;0f\033[K{ %10lld , %+5.3f ,{ %4d , %4d , %+4.3f },{ %+4.3f , %+4.3f , %+4.3f , %+4.3f }}", int64_t(temp_latest_nframe * Frame_Time), 1000000.0F / float(cmd.period), cmd.throttle_0, cmd.throttle_1, sw_angle, filt_dir, filt_x, filt_y, filt_z);

			printf("\033[10;0f\033[K{DX,DY,T}:{%+4.0f,%+4.0f,%4d}, P:{%+4.0f,%+4.0f,%+4.0f}, D:{%+4.0f,%+4.0f,%+4.0f}, I:{%+4.0f,%+4.0f,%+4.0f}, A:{%+4.0f,%+4.0f}", throttle_diff_x, throttle_diff_y, avg_throttle, -kp_xy * (filt_x - target_point[0]), -kp_xy * (filt_y - target_point[1]), -kp_z * (filt_z - target_point[2]), -kd_xy * filt_v_x, -kd_xy * filt_v_y, -kd_z * filt_v_z, -ki_xy * x_int_temp, -ki_xy * y_int_temp, -ki_z * z_int_temp, -ka_xy * filt_a_x, -ka_xy * filt_a_y);

			// print to file
			fprintf(data_log, "DATA  ,  %lld,%.3f,  %d,%d,%.3f  ,  %.3f,%.3f,%.3f  ,  %.3f,%.3f,%.3f,%.3f  ,  %.3f,%.3f,%.3f,%.3f  ,  %.3f,%.3f,%.3f,%.3f\n", int64_t(temp_latest_nframe * Frame_Time), 1000000.0F / float(cmd.period), cmd.throttle_0, cmd.throttle_1, sw_angle, x_int, y_int, z_int, filt_dir, filt_x, filt_y, filt_z, filt_v_dir, filt_v_x, filt_v_y, filt_v_z, filt_a_dir, filt_a_x, filt_a_y, filt_a_z);

			// Send to the client
			DWORD bytesWritten;

			// create a byte array with command, but prepend -1 to the array, meaning this message is broadcasted
			uint8_t cmd_array[sizeof(cmd) + 1] = { 0 };
			cmd_array[0] = uint8_t(-1);
			memcpy_s(cmd_array + 1, sizeof(cmd), cmd.raw, sizeof(cmd));

			WriteFile(hSerial, cmd_array, sizeof(cmd) + 1, &bytesWritten, NULL);
		}
	}
}

void ReceiveMessageTask()
{
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

	while (true)
	{
		COMSTAT comStat;
		DWORD errors;

		ClearCommError(hSerial, &errors, &comStat);

		if (comStat.cbInQue > 0)
		{
			int64_t tnow = get_time();

			while (get_time() - tnow <= 20)
			{
			}

			char buffer[256] = { 0 };
			DWORD bytesRead;
			auto rsucc = ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL);

			// first byte for the transmitter id, 0 is the drone
			if (bytesRead == sizeof(Motor_FB_t) + 1 && buffer[0] == 0)
			{
				Motor_FB_t spd_fb_temp;
				memcpy_s(spd_fb_temp.raw, sizeof(spd_fb_temp), buffer + 1, sizeof(spd_fb_temp));
				printf("\033[7;0f\033[K{ %10lld , %4d , %5d }", spd_fb_temp.time, spd_fb_temp.throttle, spd_fb_temp.eRPM);

				fprintf(data_log, "FB  ,  %lld,%d,%d\n", spd_fb_temp.time, spd_fb_temp.throttle, spd_fb_temp.eRPM);
			}
			else if (bytesRead == 9 && buffer[0] == 1)
			{
				static int64_t last_launch_time = 0;

				int64_t launch_time = 0;
				memcpy_s(&launch_time, sizeof(launch_time), buffer + 1, sizeof(launch_time));

				if (last_launch_time != launch_time)
				{
					printf("\033[13;0f\033[KLaunched , %lld", launch_time);

					fprintf(data_log, "Launched , %lld\n", launch_time);

					last_launch_time = launch_time;
				}
			}
			else
			{
				// printf("\033[9;0f\033[K%s", buffer);
			}
		}
	}
}