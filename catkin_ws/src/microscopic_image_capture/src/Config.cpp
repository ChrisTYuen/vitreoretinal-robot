/* -LICENSE-START-
** Copyright (c) 2013 Blackmagic Design
**  
** Permission is hereby granted, free of charge, to any person or organization 
** obtaining a copy of the software and accompanying documentation (the 
** "Software") to use, reproduce, display, distribute, sub-license, execute, 
** and transmit the Software, and to prepare derivative works of the Software, 
** and to permit third-parties to whom the Software is furnished to do so, in 
** accordance with:
** 
** (1) if the Software is obtained from Blackmagic Design, the End User License 
** Agreement for the Software Development Kit (“EULA”) available at 
** https://www.blackmagicdesign.com/EULA/DeckLinkSDK; or
** 
** (2) if the Software is obtained from any third party, such licensing terms 
** as notified by that third party,
** 
** and all subject to the following:
** 
** (3) the copyright notices in the Software and this entire statement, 
** including the above license grant, this restriction and the following 
** disclaimer, must be included in all copies of the Software, in whole or in 
** part, and all derivative works of the Software, unless such copies or 
** derivative works are solely in the form of machine-executable object code 
** generated by a source language processor.
** 
** (4) THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
** OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
** FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT 
** SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE 
** FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE, 
** ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
** DEALINGS IN THE SOFTWARE.
** 
** A copy of the Software is available free of charge at 
** https://www.blackmagicdesign.com/desktopvideo_sdk under the EULA.
** 
** -LICENSE-END-
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <unistd.h>
#include <microscopic_image_capture/Config.h>

BMDConfig::BMDConfig() :
    m_deckLinkIndex(0),
    m_displayModeIndex(-1),
	m_audioChannels(2),
	m_audioSampleDepth(16),
	m_maxFrames(-1),
	m_inputFlags(bmdVideoInputFlagDefault),
    m_pixelFormat(bmdFormat8BitYUV),
    m_timecodeFormat(),
	m_videoOutputFile(),
	m_audioOutputFile(),
	m_deckLinkName(),
	m_displayModeName()
{
}

BMDConfig::~BMDConfig()
{
	if (m_deckLinkName)
		free(m_deckLinkName);

	if (m_displayModeName)
		free(m_displayModeName);
}

bool BMDConfig::ParseArguments(int argc,  char** argv)
{
	int		ch;
	bool	displayHelp = false;

	while ((ch = getopt(argc, argv, "d:?h3c:s:v:a:m:n:p:t:")) != -1)
	{
		switch (ch)
		{
			case 'd':
				m_deckLinkIndex = atoi(optarg);
				break;

			case 'm':
				m_displayModeIndex = atoi(optarg);
				break;

			case 'c':
				m_audioChannels = atoi(optarg);
				if (m_audioChannels != 2 &&
					m_audioChannels != 8 &&
					m_audioChannels != 16)
				{
					fprintf(stderr, "Invalid argument: Audio Channels must be either 2, 8 or 16\n");
					return false;
				}
				break;

			case 's':
				m_audioSampleDepth = atoi(optarg);
				if (m_audioSampleDepth != 16 && m_audioSampleDepth != 32)
				{
					fprintf(stderr, "Invalid argument: Audio Sample Depth must be either 16 bits or 32 bits\n");
					return false;
				}
				break;

			case 'v':
				m_videoOutputFile = optarg;
				break;

			case 'a':
				m_audioOutputFile = optarg;
				break;

			case 'n':
				m_maxFrames = atoi(optarg);
				break;

			case '3':
				m_inputFlags |= bmdVideoInputDualStream3D;
				break;

			case 'p':
				switch(atoi(optarg))
				{
					case 0: m_pixelFormat = bmdFormat8BitYUV; break;
					case 1: m_pixelFormat = bmdFormat10BitYUV; break;
					case 2: m_pixelFormat = bmdFormat10BitRGB; break;
					default:
						fprintf(stderr, "Invalid argument: Pixel format %d is not valid", atoi(optarg));
						return false;
				}
				break;

			case 't':
				if (!strcmp(optarg, "rp188"))
					m_timecodeFormat = bmdTimecodeRP188Any;
				else if (!strcmp(optarg, "vitc"))
					m_timecodeFormat = bmdTimecodeVITC;
				else if (!strcmp(optarg, "serial"))
					m_timecodeFormat = bmdTimecodeSerial;
				else
				{
					fprintf(stderr, "Invalid argument: Timecode format \"%s\" is invalid\n", optarg);
					return false;
				}
				break;

			case '?':
			case 'h':
				displayHelp = true;
		}
	}

	if (m_deckLinkIndex < 0)
	{
		fprintf(stderr, "You must select a device\n");
		DisplayUsage(1);
	}

	if (m_displayModeIndex < -1)
	{
		fprintf(stderr, "You must select a display mode\n");
		DisplayUsage(1);
	}

	if (displayHelp)
		DisplayUsage(0);

	// Get device and display mode names
	IDeckLink* deckLink = GetSelectedDeckLink();
	if (deckLink != NULL)
	{
		if (m_displayModeIndex != -1)
		{
			IDeckLinkDisplayMode* displayMode = GetSelectedDeckLinkDisplayMode(deckLink);
			if (displayMode != NULL)
			{
				displayMode->GetName((const char**)&m_displayModeName);
				displayMode->Release();
			}
			else
			{
				m_displayModeName = strdup("Invalid");
			}
		}
		else
		{
			m_displayModeName = strdup("Format Detection");
		}

		deckLink->GetDisplayName((const char**)&m_deckLinkName);
		deckLink->Release();
	}
	else
	{
		m_deckLinkName = strdup("Invalid");
	}

	return true;
}

IDeckLink* BMDConfig::GetSelectedDeckLink()
{
	HRESULT				result;
	IDeckLink*			deckLink;
	IDeckLinkIterator*	deckLinkIterator = CreateDeckLinkIteratorInstance();
	int					i = m_deckLinkIndex;

	if (!deckLinkIterator)
	{
		fprintf(stderr, "This application requires the DeckLink drivers installed.\n");
		return NULL;
	}

	while((result = deckLinkIterator->Next(&deckLink)) == S_OK)
	{
		IDeckLinkProfileAttributes*	deckLinkAttributes = NULL;
		int64_t 					intAttribute;

		result = deckLink->QueryInterface(IID_IDeckLinkProfileAttributes, (void**)&deckLinkAttributes);
		if (result != S_OK)
		{
			deckLink->Release();
			break;
		}

		// Skip over devices that don't support capture
		if ((deckLinkAttributes->GetInt(BMDDeckLinkVideoIOSupport, &intAttribute) == S_OK) && 
			((intAttribute & bmdDeviceSupportsCapture) != 0))
		{
			if (i == 0)
				break;
			--i;
		}

		deckLinkAttributes->Release();
		deckLink->Release();
	}

	deckLinkIterator->Release();

	if (result != S_OK)
		return NULL;

	return deckLink;
}

IDeckLinkDisplayMode* BMDConfig::GetSelectedDeckLinkDisplayMode(IDeckLink* deckLink)
{
	HRESULT							result;
	IDeckLinkDisplayMode*			displayMode = NULL;
	IDeckLinkInput*					deckLinkInput = NULL;
	IDeckLinkDisplayModeIterator*	displayModeIterator = NULL;

	result = deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&deckLinkInput);
	if (result != S_OK)
		goto bail;

	if (m_displayModeIndex < 0)
	{
		// For format detection mode, use 1080p30 as default mode to start with
		result = deckLinkInput->GetDisplayMode(bmdModeHD1080p30, &displayMode);
		if (result != S_OK)
			goto bail;
	}
	else
	{
		int i = m_displayModeIndex;

		result = deckLinkInput->GetDisplayModeIterator(&displayModeIterator);
		if (result != S_OK)
			goto bail;

		while ((result = displayModeIterator->Next(&displayMode)) == S_OK)
		{
			if (i == 0)
				break;
			--i;

			displayMode->Release();
			displayMode = NULL;
		}
	}

bail:
	if (displayModeIterator)
		displayModeIterator->Release();

	if (deckLinkInput)
		deckLinkInput->Release();

	return displayMode;
}

void BMDConfig::DisplayUsage(int status)
{
	HRESULT							result = E_FAIL;
	IDeckLinkIterator*				deckLinkIterator = CreateDeckLinkIteratorInstance();
	IDeckLinkDisplayModeIterator*	displayModeIterator = NULL;

	IDeckLink*						deckLink = NULL;
	IDeckLink*						deckLinkSelected = NULL;
	int								deckLinkCount = 0;
	char*							deckLinkName = NULL;

	IDeckLinkProfileAttributes*		deckLinkAttributes = NULL;
	bool							formatDetectionSupported;

	IDeckLinkInput*					deckLinkInput = NULL;
	IDeckLinkDisplayMode*			displayModeUsage;
	int								displayModeCount = 0;
	char*							displayModeName;

	fprintf(stderr,
		"Usage: Capture -d <device id> -m <mode id> [OPTIONS]\n"
		"\n"
		"    -d <device id>:\n"
	);

	// Loop through all available devices
	while (deckLinkIterator->Next(&deckLink) == S_OK)
	{
		bool deckLinkActive = false;
		bool deckLinkSupportsCapture = false;

		if (deckLink->QueryInterface(IID_IDeckLinkProfileAttributes, (void**)&deckLinkAttributes) == S_OK)
		{
			int64_t intAttribute;
			deckLinkActive = ((deckLinkAttributes->GetInt(BMDDeckLinkDuplex, &intAttribute) == S_OK) && 
								(intAttribute != bmdDuplexInactive));
			
			deckLinkSupportsCapture = ((deckLinkAttributes->GetInt(BMDDeckLinkVideoIOSupport, &intAttribute) == S_OK) && 
										((intAttribute & bmdDeviceSupportsCapture) != 0));

			deckLinkAttributes->Release();
		}

		if (!deckLinkSupportsCapture)
		{
			deckLink->Release();
			continue;
		}
		
		result = deckLink->GetDisplayName((const char**)&deckLinkName);
		if (result == S_OK)
		{
			fprintf(stderr,
				"        %2d: %s%s%s\n",
				deckLinkCount,
				deckLinkName,
				deckLinkActive ? "" : " (inactive)",
				deckLinkCount == m_deckLinkIndex ? " (selected)" : ""
			);

			free(deckLinkName);
		}

		if (deckLinkCount == m_deckLinkIndex)
			deckLinkSelected = deckLink;
		else
			deckLink->Release();

		++deckLinkCount;
	}

	if (deckLinkCount == 0)
		fprintf(stderr, "        No DeckLink devices found. Is the driver loaded?\n");

	deckLinkName = NULL;

	if (deckLinkSelected != NULL)
		deckLinkSelected->GetModelName((const char**)&deckLinkName);

	fprintf(stderr,
		"    -m <mode id>: (%s)\n",
		deckLinkName ? deckLinkName : ""
	);

	if (deckLinkName != NULL)
		free(deckLinkName);

	// Loop through all available display modes on the delected DeckLink device
	if (deckLinkSelected == NULL)
	{
		fprintf(stderr, "        No DeckLink device selected\n");
		goto bail;
	}

	result = deckLinkSelected->QueryInterface(IID_IDeckLinkProfileAttributes, (void**)&deckLinkAttributes);
	if (result == S_OK)
	{
		result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &formatDetectionSupported);
		if (result == S_OK && formatDetectionSupported)
			fprintf(stderr, "        -1:  auto detect format\n");
	}

	result = deckLinkSelected->QueryInterface(IID_IDeckLinkInput, (void**)&deckLinkInput);
	if (result != S_OK)
		goto bail;

	result = deckLinkInput->GetDisplayModeIterator(&displayModeIterator);
	if (result != S_OK)
		goto bail;

	while (displayModeIterator->Next(&displayModeUsage) == S_OK)
	{
		result = displayModeUsage->GetName((const char **)&displayModeName);
		if (result == S_OK)
		{
			BMDTimeValue frameRateDuration;
			BMDTimeValue frameRateScale;

			displayModeUsage->GetFrameRate(&frameRateDuration, &frameRateScale);

			fprintf(stderr,
				"        %2d:  %-20s \t %li x %li \t %g FPS\n",
				displayModeCount,
				displayModeName,
				displayModeUsage->GetWidth(),
				displayModeUsage->GetHeight(),
				(double)frameRateScale / (double)frameRateDuration
			);

			free(displayModeName);
		}

		displayModeUsage->Release();
		++displayModeCount;
	}

bail:
	fprintf(stderr,
		"    -p <pixelformat>\n"
		"         0:  8 bit YUV (4:2:2) (default)\n"
		"         1:  10 bit YUV (4:2:2)\n"
		"         2:  10 bit RGB (4:4:4)\n"
		"    -t <format>          Print timecode\n"
		"         rp188:  RP 188\n"
		"         vitc:   VITC\n"
		"         serial: Serial Timecode\n"
		"    -v <filename>        Filename raw video will be written to\n"
		"    -a <filename>        Filename raw audio will be written to\n"
		"    -c <channels>        Audio Channels (2, 8 or 16 - default is 2)\n"
		"    -s <depth>           Audio Sample Depth (16 or 32 - default is 16)\n"
		"    -n <frames>          Number of frames to capture (default is unlimited)\n"
		"    -3                   Capture Stereoscopic 3D (Requires 3D Hardware support)\n"
		"\n"
		"Capture video and/or audio to a file. Raw video and/or audio can be viewed with mplayer eg:\n"
		"\n"
		"    Capture -d 0 -m 2 -n 50 -v video.raw -a audio.raw\n"
		"    mplayer video.raw -demuxer rawvideo -rawvideo pal:uyvy -audiofile audio.raw -audio-demuxer 20 -rawaudio rate=48000\n"
	);

	if (deckLinkIterator != NULL)
		deckLinkIterator->Release();

	if (displayModeIterator != NULL)
		displayModeIterator->Release();

	if (deckLinkInput != NULL)
		deckLinkInput->Release();

	if (deckLinkAttributes != NULL)
		deckLinkAttributes->Release();

	if (deckLinkSelected != NULL)
		deckLinkSelected->Release();

	exit(status);
}

void BMDConfig::DisplayConfiguration()
{
	fprintf(stderr, "Capturing with the following configuration:\n"
		" - Capture device: %s\n"
		" - Video mode: %s %s\n"
		" - Pixel format: %s\n"
		" - Audio channels: %u\n"
		" - Audio sample depth: %u bit \n",
		m_deckLinkName,
		m_displayModeName,
		(m_inputFlags & bmdVideoInputDualStream3D) ? "3D" : "",
		GetPixelFormatName(m_pixelFormat),
		m_audioChannels,
		m_audioSampleDepth
	);
}

const char* BMDConfig::GetPixelFormatName(BMDPixelFormat pixelFormat)
{
	switch (pixelFormat)
	{
		case bmdFormat8BitYUV:
			return "8 bit YUV (4:2:2)";
		case bmdFormat10BitYUV:
			return "10 bit YUV (4:2:2)";
		case bmdFormat10BitRGB:
			return "10 bit RGB (4:4:4)";
	}
	return "unknown";
}
