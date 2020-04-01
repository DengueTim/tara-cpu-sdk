///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, e-con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
#pragma once
#include "Tara.h"

#include <thread>
#include <future>
#include <queue>
#include <pthread.h>

class OpenCVViewer
{
public:
	//Initialise
	int Init();

private:
	struct FrameQueueStruct {
		timeval timeVal;
		cv::Mat left;
		cv::Mat right;
	};

	int ManualExposure;
	TaraRev g_eRev;
	volatile bool saveFramesAndIMU = false;
	std::mutex frameQueueMux;
	std::condition_variable frameQueueCond;
	std::queue<FrameQueueStruct *> frameQueue;

	IMUDATAOUTPUT_TypeDef *imuOutputBuffer;

	//OpenCV module to stream the Tara Rectified Images
	int TaraViewer();

	int FrameWriter(char *SequenceDirectoryBuf);
	int imuWriter(char *SequenceDirectoryBuf, pthread_mutex_t *imuDataReadyMux);

	void GetImuValueThread();

	//disparity object
	Tara::Disparity _Disparity;
};
