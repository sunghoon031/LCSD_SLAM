/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/



#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"


#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"



#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


std::string vignette = "";
std::string gammaCalib = "";
std::string source = "";
std::string calib = "";
double rescale = 1;
bool reverse = false;
bool disableROS = false;
bool half_resolution = false;
int start=0;
int end=100000;
bool preRectification = false;
float playbackSpeed=0;	// 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool useSampleOutput=false;

int mode=0;

bool firstRosSpin=false;

using namespace dso;


void my_exit_handler(int s)
{
	printf("Caught signal %d\n",s);
	exit(1);
}

void exitThread()
{
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	firstRosSpin=true;
	while(true) pause();
}



void settingsDefault(int preset)
{
	printf("\n=============== PRESET Settings: ===============\n");
	if(preset == 0 || preset == 1)
	{
		printf("DEFAULT settings:\n"
				"- %s real-time enforcing\n"
				"- 2000 active points\n"
				"- 5-7 active frames\n"
				"- 1-6 LM iteration each KF\n"
				"- original image resolution\n", preset==0 ? "no " : "1x");

		//playbackSpeed = (preset==0 ? 0 : 1);
		//preload = preset==1;
		playbackSpeed = 1;

		setting_desiredImmatureDensity = 1500;
		setting_desiredPointDensity = 2000;
		setting_minFrames = 5;
		setting_maxFrames = 7;
		setting_maxOptIterations=6;
		setting_minOptIterations=1;

		setting_logStuff = false;
	    setting_onlyLogKFPoses = true;
	    setting_render_display3D = false;
	    setting_render_displayDepth = false;
	    setting_render_displayVideo = false;
	    setting_render_displayResidual = false;
	    setting_render_renderWindowFrames = false;
	    setting_render_plotTrackingFull = false;
	    setting_render_displayCoarseTrackingFull = false;
	}

	if(preset == 2 || preset == 3)
	{
		printf("FAST settings:\n"
				"- %s real-time enforcing\n"
				"- 800 active points\n"
				"- 4-6 active frames\n"
				"- 1-4 LM iteration each KF\n"
				"- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");

		//playbackSpeed = (preset==2 ? 0 : 5);
		//preload = preset==3;
		playbackSpeed = 1;

		setting_desiredImmatureDensity = 600;
		setting_desiredPointDensity = 800;
		setting_minFrames = 4;
		setting_maxFrames = 6;
		setting_maxOptIterations=4;
		setting_minOptIterations=1;

		//benchmarkSetting_width = 424;
		//benchmarkSetting_height = 320;

		setting_logStuff = false;
        setting_onlyLogKFPoses = true;
        setting_render_display3D = false;
        setting_render_displayDepth = false;
        setting_render_displayVideo = false;
        setting_render_displayResidual = false;
        setting_render_renderWindowFrames = false;
        setting_render_plotTrackingFull = false;
        setting_render_displayCoarseTrackingFull = false;
	}

	printf("==============================================\n");
}






void parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];


    if(1==sscanf(arg,"sampleoutput=%d",&option))
    {
        if(option==1)
        {
            useSampleOutput = true;
            printf("USING SAMPLE OUTPUT WRAPPER!\n");
        }
        return;
    }

    if(1==sscanf(arg,"quiet=%d",&option))
    {
        if(option==1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }

	if(1==sscanf(arg,"preset=%d",&option))
	{
		settingsDefault(option);
		return;
	}


	if(1==sscanf(arg,"rec=%d",&option))
	{
		if(option==0)
		{
			disableReconfigure = true;
			printf("DISABLE RECONFIGURE!\n");
		}
		return;
	}



	if(1==sscanf(arg,"noros=%d",&option))
	{
		if(option==1)
		{
			disableROS = true;
			disableReconfigure = true;
			printf("DISABLE ROS (AND RECONFIGURE)!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}
	if(1==sscanf(arg,"reverse=%d",&option))
	{
		if(option==1)
		{
			reverse = true;
			printf("REVERSE!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"start=%d",&option))
	{
		start = option;
		printf("START AT %d!\n",start);
		return;
	}
	if(1==sscanf(arg,"end=%d",&option))
	{
		end = option;
		printf("END AT %d!\n",start);
		return;
	}

	if(1==sscanf(arg,"files=%s",buf))
	{
		source = buf;
		printf("loading data from %s!\n", source.c_str());
		return;
	}

	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}

	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignette = buf;
		printf("loading vignette from %s!\n", vignette.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaCalib = buf;
		printf("loading gammaCalib from %s!\n", gammaCalib.c_str());
		return;
	}

	if(1==sscanf(arg,"rescale=%f",&foption))
	{
		rescale = foption;
		printf("RESCALE %f!\n", rescale);
		return;
	}

	if(1==sscanf(arg,"speed=%f",&foption))
	{
		playbackSpeed = foption;
		printf("PLAYBACK SPEED %f!\n", playbackSpeed);
		return;
	}

	if(1==sscanf(arg,"save=%d",&option))
	{
		if(option==1)
		{
			debugSaveImages = true;
			if(42==system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			if(42==system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			if(42==system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			if(42==system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
			printf("SAVE IMAGES!\n");
		}
		return;
	}

	if(1==sscanf(arg,"mode=%d",&option))
	{

		mode = option;
		if(option==0)
		{
			printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
		}
		if(option==1)
		{
			printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
		}
		if(option==2)
		{
			printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
			setting_photometricCalibration = 0;
			setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd=3;
		}
		return;
	}

    if(1==sscanf(arg,"half=%d",&option))
    {
        if(option==1)
        {
            half_resolution = true;
            printf("HALF RESOLUTION! \n");
        }
        return;
    }

	printf("could not parse argument \"%s\"!!!!\n", arg);
}



int main( int argc, char** argv )
{
	//setlocale(LC_ALL, "");
	for(int i=1; i<argc;i++)
		parseArgument(argv[i]);

	// hook crtl+C.
	boost::thread exThread = boost::thread(exitThread);


	ImageFolderReader* reader = new ImageFolderReader(source,calib, gammaCalib, vignette);
	reader->setGlobalCalibration(half_resolution);

	if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0)
	{
		printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
		exit(1);
	}




	int lstart=start;
	int lend = end;
	int linc = 1;
	if(reverse)
	{
		printf("REVERSE!!!!");
		lstart=end-1;
		if(lstart >= reader->getNumImages())
			lstart = reader->getNumImages()-1;
		lend = start;
		linc = -1;
	}



	FullSystem* fullSystem = new FullSystem();
	fullSystem->setGammaFunction(reader->getPhotometricGamma());
	fullSystem->linearizeOperation = (playbackSpeed==0);







    IOWrap::PangolinDSOViewer* viewer = 0;
	if(!disableAllDisplay)
    {
        viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], false);
        fullSystem->outputWrapper.push_back(viewer);
    }



    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());




    // to make MacOS happy: run this in dedicated thread -- and use this one to run the GUI.
    std::thread runthread([&]() {
        std::vector<int> idsToPlay;
        std::vector<double> timesToPlayAt;
        for(int i=lstart;i>= 0 && i< reader->getNumImages() && linc*i < linc*lend;i+=linc)
        {
            idsToPlay.push_back(i);
            if(timesToPlayAt.size() == 0)
            {
                timesToPlayAt.push_back((double)0);
            }
            else
            {
                double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
                double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
                timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
            }
        }

        double sleep_init = (timesToPlayAt.at(1)-timesToPlayAt.at(0))*1e6;


        std::vector<std::shared_ptr<ImageAndExposure> > preloadedImages;
        std::vector<std::shared_ptr<MinimalImageB> > preloadedRawImages;

        int max_preloaded_images = 10000;

        if(preRectification)
        {
            printf("LOADING AND RECTIFYING (AND PHOTO-CALIBRATING IF APPLICAPABLE) ALL IMAGES!\n");
            for(int ii=0;ii<(int)idsToPlay.size(); ii++)
            {
                if (ii == max_preloaded_images) break;

                int i = idsToPlay[ii];
                std::shared_ptr<dso::ImageAndExposure> img(reader->getImage(i));
                preloadedImages.emplace_back(img);
            }
        }
        else
        {
            printf("LOADING ALL RAW IMAGES (BUT NO GEOMETRIC OR PHOTOMETRIC CALIB BEFOREHAND)!\n");
            for(int ii=0;ii<(int)idsToPlay.size(); ii++)
            {
                if (ii == max_preloaded_images) break;

                int i = idsToPlay[ii];
                std::shared_ptr<dso::MinimalImageB> img(reader->getImageRaw(i));
                preloadedRawImages.emplace_back(img);
            }
        }

        std::vector<int> idsToPlay_original(idsToPlay);

        struct timeval tv_start;
        gettimeofday(&tv_start, NULL);
        clock_t started = clock();
        double sInitializerOffset=0;
        int nSkippedFrames = 0;
        int nTotalFramesAfterInit = 0;
        std::vector<double> trackingTimes; //in ms
        double trackingStartTimestamp = 0;
        double trackingEndTimestamp = 0;
        double startTimestamp = reader->getTimestamp(idsToPlay.front());
        double endTimestamp = reader->getTimestamp(idsToPlay.back());

        double resetAllowedTimeFactor = 0.3;
        double timestampThreshold = timesToPlayAt.at((int) timesToPlayAt.size()*resetAllowedTimeFactor);

        while (!idsToPlay.empty())
        {

            if (preRectification && preloadedImages.empty())
            {
                if (!setting_debugout_runquiet)
                    printf("LOADING AND RECTIFYING (AND PHOTO-CALIBRATING IF APPLICAPABLE) ALL REMAINING IMAGES!\n");
                for(int ii=max_preloaded_images;ii<(int)idsToPlay_original.size(); ii++)
                {
                    int i = idsToPlay_original[ii];
                    std::shared_ptr<dso::ImageAndExposure> img(reader->getImage(i));
                    preloadedImages.emplace_back(img);
                }
                gettimeofday(&tv_start, NULL);
                sInitializerOffset = timesToPlayAt[idsToPlay.front()];
                if (!setting_debugout_runquiet)
                    printf("LOADING Complete!\n");
            }

            if (!preRectification && preloadedRawImages.empty())
            {
                if (!setting_debugout_runquiet)
                    printf("LOADING ALL REMAINING RAW IMAGES (BUT NO GEOMETRIC OR PHOTOMETRIC CALIB BEFOREHAND)!\n");
                for(int ii=max_preloaded_images;ii<(int)idsToPlay_original.size(); ii++)
                {
                    int i = idsToPlay_original[ii];
                    std::shared_ptr<dso::MinimalImageB> img(reader->getImageRaw(i));
                    preloadedRawImages.emplace_back(img);
                }
                gettimeofday(&tv_start, NULL);
                sInitializerOffset = timesToPlayAt[idsToPlay.front()];
                if (!setting_debugout_runquiet)
                    printf("LOADING Complete!\n");
            }



            if(!fullSystem->initialized)	// if not initialized: reset start time.
            {
                usleep((int)(sleep_init));
                gettimeofday(&tv_start, NULL);
                started = clock();
                sInitializerOffset = timesToPlayAt[idsToPlay.front()];
            }

            if(fullSystem->initFailed || setting_fullResetRequested)
            {
                  printf("RESETTING!\n");

                  std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                  delete fullSystem;

                  for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

                  fullSystem = new FullSystem();
                  fullSystem->setGammaFunction(reader->getPhotometricGamma());
                  fullSystem->linearizeOperation = (playbackSpeed==0);


                  fullSystem->outputWrapper = wraps;

                  setting_fullResetRequested=false;

                  nSkippedFrames = 0;
                  nTotalFramesAfterInit = 0;
                  trackingStartTimestamp = 0;
                  trackingTimes.clear();
            }


            bool skipFrame=false;
            double sSinceStart = 0;
            if(playbackSpeed!=0)
            {
                struct timeval tv_now; gettimeofday(&tv_now, NULL);
                sSinceStart = sInitializerOffset + ((tv_now.tv_sec-tv_start.tv_sec) + (tv_now.tv_usec-tv_start.tv_usec)/(1000.0f*1000.0f));

                if (fullSystem->initialized)
                {
                    if(sSinceStart <= timesToPlayAt[idsToPlay.front()])
                        usleep((int)((timesToPlayAt[idsToPlay.front()]-sSinceStart)*1000*1000));
                    else if(sSinceStart > timesToPlayAt[idsToPlay.front()]/*+0.5+0.1*(ii%2)*/)
                    {
                        skipFrame = true;
                        nSkippedFrames++;

                        //printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
                        double overdue_ms = (sSinceStart-timesToPlayAt[idsToPlay.front()])*1000;
                        std::cout << std::setprecision(15) << "SKIP FRAME (#"<< nSkippedFrames <<")! "<< overdue_ms << "ms overdue "<< std::endl;
                    }
                }

            }


            if(!skipFrame)
            {
                if (fullSystem->initialized)
                    nTotalFramesAfterInit++;

                struct timeval tv_beforeTracking, tv_afterTracking;

                if (preRectification)
                {
                    gettimeofday(&tv_beforeTracking, NULL);
                    fullSystem->addActiveFrame(preloadedImages.front().get(), idsToPlay.front(), half_resolution);
                    gettimeofday(&tv_afterTracking, NULL);

                }
                else
                {
                    gettimeofday(&tv_beforeTracking, NULL);
                    std::shared_ptr<dso::ImageAndExposure> img(reader->getImageFromImageRaw(preloadedRawImages.front().get(), idsToPlay.front()));
                    fullSystem->addActiveFrame(img.get(), idsToPlay.front(), half_resolution);
                    gettimeofday(&tv_afterTracking, NULL);
                }


                double trackingSecond = (tv_afterTracking.tv_sec-tv_beforeTracking.tv_sec) + (tv_afterTracking.tv_usec-tv_beforeTracking.tv_usec)/(1000.0f*1000.0f);
                double trackingMillisecond = trackingSecond*1000;
                trackingTimes.push_back(trackingMillisecond);
            }





            if(fullSystem->isLost)
            {

                if (sSinceStart < timestampThreshold)
                {
                    std::cout << "Lost within first 30%. Reset!" << std::endl;
                    setting_fullResetRequested = true;

                }
                else
                {
                    printf("LOST!!\n");
                    break;
                }
            }
            else if (fullSystem->initialized)
            {
                trackingEndTimestamp = reader->getTimestamp(idsToPlay.front());
                if (trackingStartTimestamp == 0)
                    trackingStartTimestamp = reader->getTimestamp(idsToPlay.front());
            }


            // Don't forget to erase!
            if (preRectification)
                preloadedImages.erase(preloadedImages.begin());
            else
                preloadedRawImages.erase(preloadedRawImages.begin());

            idsToPlay.erase(idsToPlay.begin());

        }


        printf("TRACKING FINISHED!!\n");
        fullSystem->blockUntilMappingIsFinished();
        clock_t ended = clock();
        struct timeval tv_end;
        gettimeofday(&tv_end, NULL);


        double trackingTimeMed, trackingTimeAvg, trackingTimeStd;
        if (trackingTimes.empty())
        {
            trackingTimeMed = 0;
            trackingTimeAvg = 0;
            trackingTimeStd = 0;
        }
        else
        {
            sort(trackingTimes.begin(), trackingTimes.end());
            trackingTimeMed = trackingTimes[trackingTimes.size()/2];
            trackingTimeAvg = accumulate(trackingTimes.begin(), trackingTimes.end(), 0.0)/trackingTimes.size();
            std::vector<double> diff(trackingTimes.size());
            std::transform(trackingTimes.begin(), trackingTimes.end(), diff.begin(), std::bind2nd(std::minus<double>(), trackingTimeAvg));
            trackingTimeStd = std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)/trackingTimes.size());
        }

        // Print result
        if (half_resolution)
            fullSystem->printResult(
                    "/home/seonghunlee/ros/dso_ros_private/catkin_ws/src/dso_ros_private/KeyFrameTrajectory_DSO_half.txt",
                    nSkippedFrames,
                    nTotalFramesAfterInit,
                    trackingTimeMed,
                    trackingTimeAvg,
                    trackingTimeStd,
                    trackingStartTimestamp,
                    trackingEndTimestamp,
                    startTimestamp,
                    endTimestamp);
        else
            fullSystem->printResult(
                    "/home/seonghunlee/ros/dso_ros_private/catkin_ws/src/dso_ros_private/KeyFrameTrajectory_DSO_full.txt",
                    nSkippedFrames,
                    nTotalFramesAfterInit,
                    trackingTimeMed,
                    trackingTimeAvg,
                    trackingTimeStd,
                    trackingStartTimestamp,
                    trackingEndTimestamp,
                    startTimestamp,
                    endTimestamp);


//        int numFramesProcessed = abs(idsToPlay[0]-idsToPlay.back());
//        double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0])-reader->getTimestamp(idsToPlay.back()));
//        double MilliSecondsTakenSingle = 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC);
//        double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
//        printf("\n======================"
//                "\n%d Frames (%.1f fps)"
//                "\n%.2fms per frame (single core); "
//                "\n%.2fms per frame (multi core); "
//                "\n%.3fx (single core); "
//                "\n%.3fx (multi core); "
//                "\n======================\n\n",
//                numFramesProcessed, numFramesProcessed/numSecondsProcessed,
//                MilliSecondsTakenSingle/numFramesProcessed,
//                MilliSecondsTakenMT / (float)numFramesProcessed,
//                1000 / (MilliSecondsTakenSingle/numSecondsProcessed),
//                1000 / (MilliSecondsTakenMT / numSecondsProcessed));
//        //fullSystem->printFrameLifetimes();
//        if(setting_logStuff)
//        {
//            std::ofstream tmlog;
//            tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
//            tmlog << 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
//                  << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) / (float)reader->getNumImages() << "\n";
//            tmlog.flush();
//            tmlog.close();
//        }

    });


    if(viewer != 0)
        viewer->run();

    runthread.join();

	for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
	{
		ow->join();
		delete ow;
	}



	printf("DELETE FULLSYSTEM!\n");
	delete fullSystem;

	printf("DELETE READER!\n");
	delete reader;

	printf("EXIT NOW!\n");
	return 0;
}
