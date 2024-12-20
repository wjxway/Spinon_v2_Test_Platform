#include "Arduino.h"
#include "TimeSync.hpp"

namespace TimeSync
{
	namespace
	{
		// when is 0th frame of optitrack in local time?
		int64_t t_offset = INT64_MAX;
		int64_t t_offset_last_calib_time = 0;

		// we compute t_offset by collecting 100 samples and take the minimum offset
		int64_t temp_t_offset = INT64_MAX;
		// temp samples count
		int temp_t_count = 0;
		// how many samples per update of t_offset
		constexpr int temp_t_count_max = 200;
		// how many samples per initial update of t_offset
		constexpr int temp_t_count_max_initial = 800;
		// inherent delay in us introduced by ESP-NOW communication
		constexpr int64_t ESPNOW_Delay = 240;

		// if synchronized
		bool synced = false;

		// handle to data queue
		QueueHandle_t data_queue;

		// if no sync within 30s, drift will reach 150us and not recoverable.
		// we trigger this task every 10s to reset the sync state if desynced
		void Desync_Task(void *pvParameters)
		{
			while (1)
			{
				vTaskDelay(pdMS_TO_TICKS(10000));
				if (t_offset_last_calib_time != 0 && esp_timer_get_time() - t_offset_last_calib_time >= 30000000LL)
				{
					synced = false;
					t_offset = INT64_MAX;
					t_offset_last_calib_time = 0;
					temp_t_offset = INT64_MAX;
					temp_t_count = 0;
				}
			}
		}
	} // anonymous namespace

	void Init()
	{
		data_queue = xQueueCreate(queue_max_len, max_data_len);

		// start the low priority desync task
		xTaskCreate(Desync_Task, "Desync_Task", 5000, nullptr, 2, NULL);
	}

	// when syncer received any message, it converts the message to raw char stream
	// and send to computer
	void Receive_Callback(const uint8_t *mac_addr, const uint8_t *data, int data_len)
	{
		// return directly if message length is incorrect
		if (data_len < sizeof(int64_t))
		{
			return;
		}

		int64_t t_now = esp_timer_get_time();

		// sync time
		int64_t t_since_start = 0;
		memcpy(&t_since_start, data + data_len - sizeof(t_since_start), sizeof(t_since_start));

		temp_t_offset = min(temp_t_offset, t_now - t_since_start);
		temp_t_count++;

		// update every temp_t_count_max samples
		// for the initial sync, we update more samples
		if ((synced && temp_t_count > temp_t_count_max) || temp_t_count > temp_t_count_max_initial)
		{
			// we update if the change in temp_t_offset is smaller than 50
			// this helps to eliminate very rare cases with extreme data.
			// for reference, the shift in clock is about 4us/s.
			constexpr int t_offset_tol = 150;
			if (!synced || abs(t_offset - temp_t_offset + ESPNOW_Delay) <= t_offset_tol)
			{
				t_offset = temp_t_offset - ESPNOW_Delay; // compensate for ESP-NOW's delay
				t_offset_last_calib_time = t_now;
				synced = true;
			}

			temp_t_count = 0;
			temp_t_offset = INT64_MAX;
		}

		// push to data if the data is not empty
		// first byte is length of data
		if (data_len != sizeof(t_since_start))
		{
			uint8_t temp_data[max_data_len] = {0};
			temp_data[0] = data_len - sizeof(t_since_start);
			memcpy(temp_data + 1, data, data_len - sizeof(t_since_start));
			xQueueSend(data_queue, temp_data, 0);
		}
	}

	int64_t Get_synced_time()
	{
		int64_t t_offset_temp;
		do
		{
			t_offset_temp = t_offset;
		} while (t_offset_temp != t_offset);

		return synced ? (esp_timer_get_time() - t_offset_temp) : 0;
	}

	int64_t Get_synced_time_offset()
	{
		int64_t t_offset_temp;
		do
		{
			t_offset_temp = t_offset;
		} while (t_offset_temp != t_offset);

		return synced ? t_offset_temp : 0;
	}

	QueueHandle_t Get_data_queue_handle()
	{
		return data_queue;
	}

	bool SyncedQ()
	{
		return synced;
	}
} // namespace TimeSync