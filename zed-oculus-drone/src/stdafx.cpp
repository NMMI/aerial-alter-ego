#include "stdafx.h"

GLchar* OVR_ZED_VS =
"#version 330 core\n \
			layout(location=0) in vec3 in_vertex;\n \
			layout(location=1) in vec2 in_texCoord;\n \
			uniform uint isLeft; \n \
			out vec2 b_coordTexture; \n \
			void main()\n \
			{\n \
				if (isLeft == 1U)\n \
				{\n \
					b_coordTexture = in_texCoord;\n \
					gl_Position = vec4(in_vertex.x, in_vertex.y, in_vertex.z,1);\n \
				}\n \
				else \n \
				{\n \
					b_coordTexture = vec2(1.0 - in_texCoord.x, in_texCoord.y);\n \
					gl_Position = vec4(-in_vertex.x, in_vertex.y, in_vertex.z,1);\n \
				}\n \
			}";

GLchar* OVR_ZED_FS =
"#version 330 core\n \
			uniform sampler2D u_textureZED; \n \
			in vec2 b_coordTexture;\n \
			out vec4 out_color; \n \
			void main()\n \
			{\n \
				out_color = vec4(texture(u_textureZED, b_coordTexture).rgb,1); \n \
			}";


int task_left(void)
{
	printf("left image thread\n");

	// Let's take a look at some orientation data.

	bool exit = false; //
	char buffer[BUF_LEN]; //buffer rcv
	int recvMsgSize; // Size of received message
	bool flag_error_size = false;

	//----------------------------------------------------------------------------UDP init start
	SOCKET s;
	struct sockaddr_in server, si_other;
	int slen;
	WSADATA wsa;

	slen = sizeof(si_other);
	//Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		return -1;
	}
	printf("Initialised.\n");

	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d", WSAGetLastError());
	}
	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT_LEFT);

	//Bind
	if (bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		return -1;
	}
	puts("Bind done");
	//----------------------------------------------------------------------------UDP init end

	bool set = false;
	char msg[10];
	//just to check the communication
	while (!set)
	{

		if (recvfrom(s, msg, 100, 0, (struct sockaddr *) &si_other, &slen) > 0)
		{
			set = true;
		}

	}

	boost::posix_time::ptime t, now;
	boost::posix_time::microseconds period = boost::posix_time::microseconds(10000);//7407 per 130fps

	t = boost::posix_time::microsec_clock::local_time();
	t = t + period;


	while (exit == false)
	{


		if (GetAsyncKeyState(VK_ESCAPE))
		{
			exit = true;
		}
		/* do useful work */
		//--------------------------------------------------------------------------------------------------START job
		//clear the buffer by filling null, it might have previously received data
		memset(buffer, '\0', BUF_LEN);


		// Block until receive message from a client
		do {
			recvMsgSize = recvfrom(s, buffer, BUF_LEN, 0, (struct sockaddr *) &si_other, &slen);
			if (recvMsgSize == SOCKET_ERROR)
			{
				printf("recvfrom() failed with error code : %d", WSAGetLastError());
				return -1;
			}

		} while (recvMsgSize > sizeof(int));
		int total_pack = ((int *)buffer)[0];

		memset(buffer, '\0', BUF_LEN);
		flag_error_size = false;
		//std::cout << "expecting length of packs:" << total_pack << std::endl;
		char * longbuf = new char[PACK_SIZE * total_pack];
		for (int i = 0; i < total_pack; i++)
		{
			recvMsgSize = recvfrom(s, buffer, BUF_LEN, 0, (struct sockaddr *) &si_other, &slen);
			if (recvMsgSize == SOCKET_ERROR)
			{
				printf("recvfrom() failed with error code : %d", WSAGetLastError());
				return -1;
			}
			if (recvMsgSize != PACK_SIZE) {
				std::cerr << "Received unexpected size pack:" << recvMsgSize << std::endl;
				std::cout << "index: " << i << " total pack: " << total_pack << std::endl;
				//continue;
				flag_error_size = true;
				break;

			}
			memcpy(&longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
		}

		//mutex start
		image_mtx_L_.lock();
		if (flag_error_size)
		{
			flag_image_L_ = 0;
		}
		else
		{
			cv::Mat rawData = cv::Mat(1, PACK_SIZE * total_pack, CV_8UC4, longbuf);

			//Decode image
			frame_left_ = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
			//check image
			if (frame_left_.size().width == 0) {
				std::cerr << "decode failure!" << std::endl;
				flag_image_L_ = 0;
			}
			else
			{
				flag_image_L_ = 1;
				/*cv::imshow("left_camera", frame_left_);
				cv::waitKey(1);*/
				//std::cout << "sono qui" << std::endl;
			}

		}
		image_mtx_L_.unlock();
		free(longbuf);
		//mutex end

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////end job
		now = boost::posix_time::microsec_clock::local_time();
		//Sleep function is commented because rcv function blocks the execution
		/*if (t < now)
		{
		std::cout << "task 1 !!!" << std::endl;

		}
		else
		{
		boost::posix_time::time_duration msdiff = t - now;
		boost::this_thread::sleep(msdiff);
		}

		t = t + period;*/
	}



	flag_left = true;

	closesocket(s);
	WSACleanup();
	// quit
	return 0;
}

int task_right(void)
{



	printf("Go EGO robot!\n");

	// Let's take a look at some orientation data.

	bool exit = false;
	char buffer[BUF_LEN];
	int recvMsgSize; // Size of received message
	bool flag_error_size = false;
	printf("Stupido EGO!\n");

	//----------------------------------------------------------------------------UDP init INIZIO
	SOCKET s;
	struct sockaddr_in server, si_other;
	int slen;
	WSADATA wsa;

	slen = sizeof(si_other);
	//Initialise winsock
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		return -1;
	}
	printf("Initialised.\n");

	//Create a socket
	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
	{
		printf("Could not create socket : %d", WSAGetLastError());
	}
	printf("Socket created.\n");

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT_RIGHT);

	//Bind
	if (bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		return -1;
	}
	puts("Bind done");
	//----------------------------------------------------------------------------UDP init FINE


	bool set = false;
	char msg[10];
	printf("Stupido ME!\n");
	while (!set)
	{

		if (recvfrom(s, msg, 100, 0, (struct sockaddr *) &si_other, &slen) > 0)
		{
			set = true;
		}

	}

	boost::posix_time::ptime t, now;
	boost::posix_time::microseconds period = boost::posix_time::microseconds(10000);//7407 per 130fps

	t = boost::posix_time::microsec_clock::local_time();
	t = t + period;


	while (exit == false)
	{


		if (GetAsyncKeyState(VK_ESCAPE))
		{
			exit = true;
		}
		/* do useful work */
		//---------------------------------------------------------------------------------------------------START job
		//clear the buffer by filling null, it might have previously received data
		memset(buffer, '\0', BUF_LEN);


		// Block until receive message from a client
		do {
			recvMsgSize = recvfrom(s, buffer, BUF_LEN, 0, (struct sockaddr *) &si_other, &slen);
			if (recvMsgSize == SOCKET_ERROR)
			{
				printf("recvfrom() failed with error code : %d", WSAGetLastError());
				return -1;
			}

		} while (recvMsgSize > sizeof(int));
		int total_pack = ((int *)buffer)[0];
		printf("MAledEGO!\n");
		memset(buffer, '\0', BUF_LEN);
		flag_error_size = false;
		//std::cout << "expecting length of packs:" << total_pack << std::endl;
		char * longbuf = new char[PACK_SIZE * total_pack];
		for (int i = 0; i < total_pack; i++)
		{
			recvMsgSize = recvfrom(s, buffer, BUF_LEN, 0, (struct sockaddr *) &si_other, &slen);
			if (recvMsgSize == SOCKET_ERROR)
			{
				printf("recvfrom() failed with error code : %d", WSAGetLastError());
				return -1;
			}
			if (recvMsgSize != PACK_SIZE) {
				std::cerr << "Received unexpected size pack:" << recvMsgSize << std::endl;
				std::cout << "index: " << i << " total pack: " << total_pack << std::endl;
				//continue;
				flag_error_size = true;
				break;

			}
			memcpy(&longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
		}
		printf("Son fuEGO!\n");
		//mutex start
		image_mtx_R_.lock();
		if (flag_error_size)
		{
			flag_image_R_ = 0;
		}
		else
		{
			cv::Mat rawData = cv::Mat(1, PACK_SIZE * total_pack, CV_8UC4, longbuf);


			frame_right_ = imdecode(rawData, CV_LOAD_IMAGE_COLOR);
			if (frame_right_.size().width == 0) {
				std::cerr << "decode failure!" << std::endl;
				flag_image_R_ = 0;
			}
			else
			{
				flag_image_R_ = 1;
				/*cv::imshow("right_camera", frame_right_);
				cv::waitKey(1);*/
				//std::cout << "sono qui" << std::endl;
			}

		}
		image_mtx_R_.unlock();
		free(longbuf);
		//mutex end

		//---------------------------------------------------------------------------------------------------------------------------end job
		now = boost::posix_time::microsec_clock::local_time();

		/*if (t < now)
		{
		std::cout << "task 1 !!!" << std::endl;

		}
		else
		{
		boost::posix_time::time_duration msdiff = t - now;
		boost::this_thread::sleep(msdiff);
		}

		t = t + period;*/
	}



	flag_right = true;

	closesocket(s);
	WSACleanup();
	// quit
	return 0;
}
int task1(void)
{
	bool uscita = false;
	cv::Mat left_frame(zedHeight, zedWidth, CV_8UC4);
	cv::Mat right_frame(zedHeight, zedWidth, CV_8UC4);

	//-------------------------------------------------------------------------------------------------------------------- Oculus environment
	int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;
	int winWidth =  1280;
	int winHeight = 720;
	Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
	// Create SDL2 Window
	SDL_Window* window = SDL_CreateWindow("ZED Stereo Passthrough", x, y, winWidth, winHeight, flags);
	// Create OpenGL context
	SDL_GLContext glContext = SDL_GL_CreateContext(window);
	// Initialize GLEW
	glewInit();
	// Turn off vsync to let the compositor do its magic
	SDL_GL_SetSwapInterval(0);

	//sl::uchar4 dark_bckgrd(44, 44, 44, 255);
	GLuint zedTextureID[2];
	glGenTextures(2, zedTextureID);
	for (int eye = 0; eye < 2; eye++) {
		// Generate OpenGL texture
		glBindTexture(GL_TEXTURE_2D, zedTextureID[eye]);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	}


	float pixel_density = 1.75f;
	ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
	// Get the texture sizes of Oculus eyes
	ovrSizei textureSize0 = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], pixel_density);
	ovrSizei textureSize1 = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], pixel_density);
	// Compute the final size of the render buffer
	ovrSizei bufferSize;
	bufferSize.w = textureSize0.w + textureSize1.w;
	bufferSize.h = std::max(textureSize0.h, textureSize1.h);

	// Initialize OpenGL swap textures to render
	ovrTextureSwapChain textureChain = nullptr;

	// Description of the swap chain
	ovrTextureSwapChainDesc descTextureSwap = {};
	descTextureSwap.Type = ovrTexture_2D;
	descTextureSwap.ArraySize = 1;
	descTextureSwap.Width = bufferSize.w;
	descTextureSwap.Height = bufferSize.h;
	descTextureSwap.MipLevels = 1;
	descTextureSwap.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
	descTextureSwap.SampleCount = 1;
	descTextureSwap.StaticImage = ovrFalse;
	// Create the OpenGL texture swap chain
	result = ovr_CreateTextureSwapChainGL(session, &descTextureSwap, &textureChain);

	ovrErrorInfo errInf;
	if (OVR_SUCCESS(result)) {
		int length = 0;
		ovr_GetTextureSwapChainLength(session, textureChain, &length);
		for (int i = 0; i < length; ++i) {
			GLuint chainTexId;
			ovr_GetTextureSwapChainBufferGL(session, textureChain, i, &chainTexId);
			glBindTexture(GL_TEXTURE_2D, chainTexId);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		}
	}
	else {
		ovr_GetLastErrorInfo(&errInf);
		std::cout << "ERROR: failed creating swap texture " << errInf.ErrorString << std::endl;
		ovr_Destroy(session);
		ovr_Shutdown();
		SDL_GL_DeleteContext(glContext);
		SDL_DestroyWindow(window);
		SDL_Quit();
		return -1;
	}

	// Generate frame buffer to render
	GLuint fboID;
	glGenFramebuffers(1, &fboID);
	// Generate depth buffer of the frame buffer
	GLuint depthBuffID;
	glGenTextures(1, &depthBuffID);
	glBindTexture(GL_TEXTURE_2D, depthBuffID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	GLenum internalFormat = GL_DEPTH_COMPONENT24;
	GLenum type = GL_UNSIGNED_INT;
	glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, type, NULL);
	// Create a mirror texture to display the render result in the SDL2 window
	ovrMirrorTextureDesc descMirrorTexture;
	memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
	descMirrorTexture.Width = winWidth;
	descMirrorTexture.Height = winHeight;
	descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

	ovrMirrorTexture mirrorTexture = nullptr;
	result = ovr_CreateMirrorTextureGL(session, &descMirrorTexture, &mirrorTexture);
	if (!OVR_SUCCESS(result)) {
		ovr_GetLastErrorInfo(&errInf);
		std::cout << "ERROR: Failed to create mirror texture " << errInf.ErrorString << std::endl;
	}
	GLuint mirrorTextureId;
	ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &mirrorTextureId);

	GLuint mirrorFBOID;
	glGenFramebuffers(1, &mirrorFBOID);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
	glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTextureId, 0);
	glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
	// Frame index used by the compositor, it needs to be updated each new frame
	long long frameIndex = 0;

	// FloorLevel will give tracking poses where the floor height is 0
	ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);

	// Initialize a default Pose
	ovrPosef eyeRenderPose[2];

	// Get the render description of the left and right "eyes" of the Oculus headset
	ovrEyeRenderDesc eyeRenderDesc[2];
	eyeRenderDesc[0] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0]);
	eyeRenderDesc[1] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1]);

	// Get the Oculus view scale description
	//ovrVector3f hmdToEyeOffset[2];
	ovrPosef hmdToEyeOffset[2];
	double sensorSampleTime;

	// Create and compile the shader's sources
	Shader shader(OVR_ZED_VS, OVR_ZED_FS);

	// Compute the useful part of the ZED image
	unsigned int widthFinal = bufferSize.w / 2;
	float heightGL = 1.f;
	float widthGL = 1.f;
	if (zedWidth > 0.f) {
		unsigned int heightFinal = zedHeight * widthFinal / (float)zedWidth;
		// Convert this size to OpenGL viewport's frame's coordinates
		heightGL = (heightFinal) / (float)(bufferSize.h);
		widthGL = ((zedWidth * (heightFinal / (float)zedHeight)) / (float)widthFinal);
	}
	else {
		std::cout << "WARNING: ZED parameters got wrong values."
			"Default vertical and horizontal FOV are used.\n"
			"Check your calibration file or check if your ZED is not too close to a surface or an object."
			<< std::endl;
	}

	// Compute the Horizontal Oculus' field of view with its parameters
	float ovrFovH = (atanf(hmdDesc.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc.DefaultEyeFov[0].RightTan));
	// Compute the Vertical Oculus' field of view with its parameters
	float ovrFovV = (atanf(hmdDesc.DefaultEyeFov[0].UpTan) + atanf(hmdDesc.DefaultEyeFov[0].DownTan));

	// Compute the center of the optical lenses of the headset
	float offsetLensCenterX = ((atanf(hmdDesc.DefaultEyeFov[0].LeftTan)) / ovrFovH) * 2.f - 1.f;
	float offsetLensCenterY = ((atanf(hmdDesc.DefaultEyeFov[0].UpTan)) / ovrFovV) * 2.f - 1.f;
	
	// Create a rectangle with the computed coordinates and push it in GPU memory
	struct GLScreenCoordinates {
		float left, up, right, down;
	} screenCoord;

	screenCoord.up = heightGL + offsetLensCenterY;
	screenCoord.down = heightGL - offsetLensCenterY;
	screenCoord.right = widthGL + offsetLensCenterX;
	screenCoord.left = widthGL - offsetLensCenterX;

	float rectVertices[12] = { -screenCoord.left, -screenCoord.up, 0, screenCoord.right, -screenCoord.up, 0, screenCoord.right, screenCoord.down, 0, -screenCoord.left, screenCoord.down, 0 };
	GLuint rectVBO[3];
	glGenBuffers(1, &rectVBO[0]);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertices), rectVertices, GL_STATIC_DRAW);

	float rectTexCoord[8] = { 0, 1, 1, 1, 1, 0, 0, 0 };
	glGenBuffers(1, &rectVBO[1]);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectTexCoord), rectTexCoord, GL_STATIC_DRAW);

	unsigned int rectIndices[6] = { 0, 1, 2, 0, 2, 3 };
	glGenBuffers(1, &rectVBO[2]);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectIndices), rectIndices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Initialize hit value
	float hit = 0.02f;
	// Initialize a boolean that will be used to stop the application�s loop and another one to pause/unpause rendering
	bool end = false;
	// SDL variable that will be used to store input events
	SDL_Event events;
	// This boolean is used to test if the application is focused
	bool isVisible = true;
	bool rcv_R_image, rcv_L_image;
	rcv_R_image = rcv_L_image = false;

	// Enable the shader
	glUseProgram(shader.getProgramId());
	// Bind the Vertex Buffer Objects of the rectangle that displays ZED images

	// vertices
	glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
	glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
	// indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
	// texture coordinates
	glEnableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
	glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
	glVertexAttribPointer(Shader::ATTRIB_TEXTURE2D_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);

	//--------------------------------------------------------------------------------------------------------------------fine ambiente oculus

	boost::posix_time::ptime t, now, start_time;
	boost::posix_time::microseconds period = boost::posix_time::microseconds(16000); //60 HZ
	t = boost::posix_time::microsec_clock::local_time();
	t = t + period;
	///////////////////////////
	int count_fps = 0;
	boost::posix_time::time_duration msdiff_fps;
	start_time = boost::posix_time::microsec_clock::local_time();
	//////////////////////////

	while (uscita == false)
	{
		if (GetAsyncKeyState(VK_ESCAPE))
		{
			uscita = true;
		}

		/* do useful work */


		//----------------------------------------------------------------------------------------------------------------------------------------start job
		//Copy images Left/Right
		image_mtx_L_.lock();
		if (flag_image_L_)
		{
			frame_left_.copyTo(left_frame);
			rcv_L_image = true;
			
		}
		else rcv_L_image = false;
		image_mtx_L_.unlock();
		image_mtx_R_.lock();
		if (flag_image_R_)
		{
			frame_right_.copyTo(right_frame);
			rcv_R_image = true;
		}
		else rcv_R_image = false;
		image_mtx_R_.unlock();
	
		// Get texture swap index where we must draw our frame
		GLuint curTexId;
		int curIndex;
		ovr_GetTextureSwapChainCurrentIndex(session, textureChain, &curIndex);
		ovr_GetTextureSwapChainBufferGL(session, textureChain, curIndex, &curTexId);

		// Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
		hmdToEyeOffset[ovrEye_Left] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[ovrEye_Left]).HmdToEyePose;
		hmdToEyeOffset[ovrEye_Right] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[ovrEye_Right]).HmdToEyePose;
		// Get eye poses, feeding in correct IPD offset
		ovr_GetEyePoses(session, frameIndex, ovrTrue, hmdToEyeOffset, eyeRenderPose, &sensorSampleTime);

		// If the application is focused
		if (isVisible)
		{
			// If successful grab a new ZED image
			if (rcv_R_image && rcv_L_image)
			{
				// Update the ZED frame counter
				//zedc++;
				
				// Bind the frame buffer
				glBindFramebuffer(GL_FRAMEBUFFER, fboID);
				// Set its color layer 0 as the current swap texture
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
				// Set its depth layer as our depth buffer
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID, 0);
				// Clear the frame buffer
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glClearColor(0, 0, 0, 1);

				// Render for each Oculus eye the equivalent ZED image
				for (int eye = 0; eye < 2; eye++)
				{
					// Set the left or right vertical half of the buffer as the viewport
					glViewport(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
					// Bind the left or right ZED image
					glBindTexture(GL_TEXTURE_2D, eye == ovrEye_Left ? zedTextureID[ovrEye_Left] : zedTextureID[ovrEye_Right]);
					////////////

					/*unsigned char* image_left = left_frame.data;
					unsigned char* image_right = right_frame.data;*/
					if (eye == ovrEye_Left)
					{
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, zedWidth, zedHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, left_frame.ptr());
					}
					else
					{
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, zedWidth, zedHeight, 0, GL_BGR, GL_UNSIGNED_BYTE, right_frame.ptr());
					}

					//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, sl::zed::cvMat2slMat(eye == ovrEye_Left ? left_frame : right_frame).data);
					//////////////
					// Bind the hit value
					glUniform1f(glGetUniformLocation(shader.getProgramId(), "hit"), eye == ovrEye_Left ? hit : -hit);
					// Bind the isLeft value
					glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"), eye == ovrEye_Left ? 1U : 0U);
					// Draw the ZED image
					glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
				}

				// Avoids an error when calling SetAndClearRenderSurface during next iteration.
				// Without this, during the next while loop iteration SetAndClearRenderSurface
				// would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
				// associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
				glBindFramebuffer(GL_FRAMEBUFFER, fboID);
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
				glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
				// Commit changes to the textures so they get picked up frame
				ovr_CommitTextureSwapChain(session, textureChain);
			

				// Do not forget to increment the frameIndex!
				frameIndex++;
			}
		}
		/*
		Note: Even if we don't ask to refresh the framebuffer or if the Camera::grab()
		doesn't catch a new frame, we have to submit an image to the Rift; it
		needs 75Hz refresh. Else there will be jumbs, black frames and/or glitches
		in the headset.
		*/

		ovrLayerEyeFov ld;
		ld.Header.Type = ovrLayerType_EyeFov;
		// Tell to the Oculus compositor that our texture origin is at the bottom left
		ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL | Disable head tracking
																	// Set the Oculus layer eye field of view for each view
		for (int eye = 0; eye < 2; ++eye)
		{
			// Set the color texture as the current swap texture
			ld.ColorTexture[eye] = textureChain;
			// Set the viewport as the right or left vertical half part of the color texture
			ld.Viewport[eye] = OVR::Recti(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
			// Set the field of view
			ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
			// Set the pose matrix
			ld.RenderPose[eye] = eyeRenderPose[eye];
		}

		ld.SensorSampleTime = sensorSampleTime;

		ovrLayerHeader* layers = &ld.Header;
		// Submit the frame to the Oculus compositor
		// which will display the frame in the Oculus headset
		result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);

		if (!OVR_SUCCESS(result))
		{
			std::cout << "ERROR: failed to submit frame" << std::endl;
			glDeleteBuffers(3, rectVBO);
			ovr_DestroyTextureSwapChain(session, textureChain);
			ovr_DestroyMirrorTexture(session, mirrorTexture);
			ovr_Destroy(session);
			ovr_Shutdown();
			SDL_GL_DeleteContext(glContext);
			SDL_DestroyWindow(window);
			SDL_Quit();
			return 1;
		}

		if (result == ovrSuccess && !isVisible)
		{
			std::cout << "The application is now shown in the headset." << std::endl;
		}
		isVisible = (result == ovrSuccess);

		// This is not really needed for this application but it may be usefull for an more advanced application
		ovrSessionStatus sessionStatus;
		ovr_GetSessionStatus(session, &sessionStatus);
		if (sessionStatus.ShouldRecenter)
		{
			std::cout << "Recenter Tracking asked by Session" << std::endl;
			ovr_RecenterTrackingOrigin(session);
		}

		// Copy the frame to the mirror buffer
		// which will be drawn in the SDL2 image
		glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		GLint w = winWidth;
		GLint h = winHeight;
		glBlitFramebuffer(0, h, w, 0,
			0, 0, w, h,
			GL_COLOR_BUFFER_BIT, GL_NEAREST);
		glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
		// Swap the SDL2 window
		SDL_GL_SwapWindow(window);
		///-------------------------------------------------------------------------------------------------------------------------------------------------------end job

		//////////////////_____________________________________________________________________________________________________________________________________
		//count_fps++;
		now = boost::posix_time::microsec_clock::local_time();
		//msdiff_fps = now - start_time;
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*if (msdiff_fps.total_microseconds() >= 1000000)
		{
		start_time = now;
		std::cout << "FPS: " << count_fps << "  " << "time diff: " << msdiff_fps.total_milliseconds() << std::endl;
		count_fps = 0;
		}*/
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//cv::waitKey(1);
		if (t < now)
		{
			std::cout << "task 1 !!!" << std::endl;
		////std::cout << t - now << std::endl;
		}
		else
		{
			boost::posix_time::time_duration msdiff = t - now;
			boost::this_thread::sleep(msdiff);
		}

		t = t + period;

	}

	// Disable all OpenGL buffer
	glDisableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
	glDisableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glUseProgram(0);
	glBindVertexArray(0);
	// Delete the Vertex Buffer Objects of the rectangle
	glDeleteBuffers(3, rectVBO);
	// Delete SDL, OpenGL, Oculus and ZED context
	ovr_DestroyTextureSwapChain(session, textureChain);
	ovr_DestroyMirrorTexture(session, mirrorTexture);
	ovr_Destroy(session);
	ovr_Shutdown();
	SDL_GL_DeleteContext(glContext);
	SDL_DestroyWindow(window);
	SDL_Quit();

	flag1 = 1;
	return 0;
}

int task2(void)
{	
	if (result == ovrSuccess)
	{
		//----------------------------------------------------------------------------rosserial init start
		//char *ros_master = "192.168.0.77:11411";
		//char *ros_master = "172.16.0.150:11411";
		//char *ros_master = "192.168.0.150:11411";
		char* ros_master = "192.168.1.80:11311";

		//Rosserial server library http://wiki.ros.org/rosserial_windows/
		ros::NodeHandle nh;
		printf("Connecting to server at %s\n", ros_master);

		nh.initNode(ros_master);

		//geometry_msgs::Quaternion hd_msg;
		geometry_msgs::Pose left_msg, right_msg, hd_msg;
		std_msgs::Float64 LH_ind_msg, RH_ind_msg, LH_hand_msg, RH_hand_msg, LH_Thumbstick_x_msg, LH_Thumbstick_y_msg, RH_Thumbstick_x_msg, RH_Thumbstick_y_msg;
		std_msgs::Bool Button_A_msg, Button_B_msg, Button_X_msg, Button_Y_msg, Touch_LIndexTrigger_msg, Touch_RIndexTrigger_msg, LH_Thumbstick_button_msg;

		//LEFT
		//HAND TRACKING
		ros::Publisher LH_pub("LH_track", &left_msg);
		nh.advertise(LH_pub);
		//INDEX TRIGGER
		ros::Publisher LH_ind_pub{ "LH_ind", &LH_ind_msg };
		nh.advertise(LH_ind_pub);
		//Hand TRIGGER
		ros::Publisher LH_hand_pub{ "LH_hand", &LH_hand_msg };
		nh.advertise(LH_hand_pub);
		//LEFT JOYSTICK
		ros::Publisher LH_joy_x_pub{ "LH_joy_x", &LH_Thumbstick_x_msg };
		nh.advertise(LH_joy_x_pub);
		ros::Publisher LH_joy_y_pub{ "LH_joy_y", &LH_Thumbstick_y_msg };
		nh.advertise(LH_joy_y_pub);
		ros::Publisher LH_joy_button_pub{ "LH_joy_button", &LH_Thumbstick_button_msg };
		nh.advertise(LH_joy_button_pub);
		//Left touch index trigger
		ros::Publisher Touch_LIndexTrigger_pub{ "LH_trig_touch", &Touch_LIndexTrigger_msg };
		nh.advertise(Touch_LIndexTrigger_pub);
		
		//RIGHT
		//HAND TRACKING
		ros::Publisher RH_pub("RH_track", &right_msg);
		nh.advertise(RH_pub);
		//INDEX TRIGGER
		ros::Publisher RH_ind_pub{ "RH_ind", &RH_ind_msg };
		nh.advertise(RH_ind_pub);
		//Hand TRIGGER
		ros::Publisher RH_hand_pub{ "RH_hand", &RH_hand_msg };
		nh.advertise(RH_hand_pub);
		//RIGHT JOYSTICK
		ros::Publisher RH_joy_x_pub{ "RH_joy_x", &RH_Thumbstick_x_msg };
		nh.advertise(RH_joy_x_pub);
		ros::Publisher RH_joy_y_pub{ "RH_joy_y", &RH_Thumbstick_y_msg };
		nh.advertise(RH_joy_y_pub);
		//Button A
		ros::Publisher Button_A_pub{ "Button_A", &Button_A_msg };
		nh.advertise(Button_A_pub);
		//Button B
		ros::Publisher Button_B_pub{ "Button_B", &Button_B_msg };
		nh.advertise(Button_B_pub);
		//Button X
		ros::Publisher Button_X_pub{ "Button_X", &Button_X_msg };
		nh.advertise(Button_X_pub);
		//Button Y
		ros::Publisher Button_Y_pub{ "Button_Y", &Button_Y_msg };
		nh.advertise(Button_Y_pub);
		//Right touch index trigger
		ros::Publisher Touch_RIndexTrigger_pub{ "RH_trig_touch", &Touch_RIndexTrigger_msg };
		nh.advertise(Touch_RIndexTrigger_pub);

		//Head orientation TRACKING
		ros::Publisher HD_pub("Head_track", &hd_msg);
		nh.advertise(HD_pub);		

		printf("Go EGO robot!\n");
		//----------------------------------------------------------------------------rosserial init end

		// Let's take a look at some orientation data.
		bool exit = false;
		bool flag_reset_track = true;

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		ovrTrackingState ts;
		ovrInputState inputState;	// Controller buttons

		Eigen::Quaterniond Q_left(1, 0, 0, 0);
		Eigen::Quaterniond Q_right(1, 0, 0, 0);
		Eigen::Quaterniond Q_head(1, 0, 0, 0);

		// quaternione desiderato visto dal sensor frame dell'oculus
		Eigen::Quaterniond quat_d;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		boost::posix_time::ptime t, now;
		boost::posix_time::microseconds period = boost::posix_time::microseconds(20000);//7407 per 130fps

		t = boost::posix_time::microsec_clock::local_time();
		t = t + period;


		while (exit == false)
		{

			if (GetAsyncKeyState(VK_ESCAPE))
			{
				exit = true;
			}
			/* do useful work */
			//-------------------------------------------------------------------------------------------------------START job

			//HEAD
			ts = ovr_GetTrackingState(session, 0, true);

			//HEAD
			ovrVector3f HeadPos = ts.HeadPose.ThePose.Position;
			ovrQuatf HeadOrient = ts.HeadPose.ThePose.Orientation;
			Q_head.w() = HeadOrient.w;
			Q_head.vec() << HeadOrient.x, HeadOrient.y, HeadOrient.z;

			//HANDS
			ovrVector3f LHpos = ts.HandPoses[leftHand].ThePose.Position;
			ovrQuatf LHorient = ts.HandPoses[leftHand].ThePose.Orientation;
			Q_left.w() = LHorient.w;
			Q_left.vec() << LHorient.x, LHorient.y, LHorient.z;

			ovrVector3f RHpos = ts.HandPoses[rightHand].ThePose.Position;
			ovrQuatf RHorient = ts.HandPoses[rightHand].ThePose.Orientation;
			Q_right.w() = RHorient.w;
			Q_right.vec() << RHorient.x, RHorient.y, RHorient.z;

			hd_msg.position.x = HeadPos.x;
			hd_msg.position.y = HeadPos.y;
			hd_msg.position.z = HeadPos.z;

			hd_msg.orientation.w = Q_head.w();
			hd_msg.orientation.x = Q_head.x();
			hd_msg.orientation.y = Q_head.y();
			hd_msg.orientation.z = Q_head.z();
			
			left_msg.position.x = LHpos.x;
			left_msg.position.y = LHpos.y;
			left_msg.position.z = LHpos.z;

			left_msg.orientation.w = Q_left.w();
			left_msg.orientation.x = Q_left.x();
			left_msg.orientation.y = Q_left.y();
			left_msg.orientation.z = Q_left.z();

			right_msg.position.x = RHpos.x;
			right_msg.position.y = RHpos.y;
			right_msg.position.z = RHpos.z;

			right_msg.orientation.w = Q_right.w();
			right_msg.orientation.x = Q_right.x();
			right_msg.orientation.y = Q_right.y();
			right_msg.orientation.z = Q_right.z();

			// Button controls
			if (OVR_SUCCESS(ovr_GetInputState(session, ovrControllerType_Touch, &inputState)))
			{
				if (inputState.IndexTrigger[leftHand] > 0.01f)
				{
					LH_ind_msg.data = inputState.IndexTrigger[leftHand];
					LH_ind_pub.publish(&LH_ind_msg);
				}
				
				if (inputState.IndexTrigger[rightHand] > 0.01f)
				{
					RH_ind_msg.data = inputState.IndexTrigger[rightHand];
					RH_ind_pub.publish(&RH_ind_msg);
				}

				if (inputState.HandTrigger[leftHand] > 0.01f)
				{
					LH_hand_msg.data = inputState.HandTrigger[leftHand];
					LH_hand_pub.publish(&LH_hand_msg);
				}

				if (inputState.HandTrigger[rightHand] > 0.01f)
				{
					RH_hand_msg.data = inputState.HandTrigger[rightHand];
					RH_hand_pub.publish(&RH_hand_msg);
				}
				if (inputState.Thumbstick[leftHand].x)
				{
					LH_Thumbstick_x_msg.data = inputState.Thumbstick[leftHand].x;
					LH_joy_x_pub.publish(&LH_Thumbstick_x_msg);

				}
				if (inputState.Thumbstick[leftHand].y)
				{
					LH_Thumbstick_y_msg.data = inputState.Thumbstick[leftHand].y;
					LH_joy_y_pub.publish(&LH_Thumbstick_y_msg);

				}
				if (inputState.Thumbstick[rightHand].x)
				{
					RH_Thumbstick_x_msg.data = inputState.Thumbstick[rightHand].x;
					RH_joy_x_pub.publish(&RH_Thumbstick_x_msg);

				}
				if (inputState.Thumbstick[rightHand].y)
				{
					RH_Thumbstick_y_msg.data = inputState.Thumbstick[rightHand].y;
					RH_joy_y_pub.publish(&RH_Thumbstick_y_msg);

				}
				if (inputState.Buttons & ovrButton_A)
				{
					Button_A_msg.data = true;
				}
				else Button_A_msg.data = false;
				if (inputState.Buttons & ovrButton_B)
				{
					if (flag_reset_track)
					{
						ovr_RecenterTrackingOrigin(session);
						flag_reset_track = false;
					}
					Button_B_msg.data = true;
				}
				else Button_B_msg.data = false;
				if (inputState.Buttons & ovrButton_X)
				{
					Button_X_msg.data = true;
				}
				else Button_X_msg.data = false;
				if (inputState.Buttons & ovrButton_Y)
				{
					Button_Y_msg.data = true;
				}
				else Button_Y_msg.data = false;

				if (inputState.Buttons & ovrButton_LThumb)
				{
					LH_Thumbstick_button_msg.data = true;
				}
				else LH_Thumbstick_button_msg.data = false;

				if (inputState.Touches & ovrTouch_LIndexTrigger)
				{

					Touch_LIndexTrigger_msg.data = true;
					Touch_LIndexTrigger_pub.publish(&Touch_LIndexTrigger_msg);
				}

				if (inputState.Touches & ovrTouch_RIndexTrigger)
				{

					Touch_RIndexTrigger_msg.data = true;
					Touch_RIndexTrigger_pub.publish(&Touch_RIndexTrigger_msg);
				}

			}

			if (!flag_reset_track)
			{
				LH_pub.publish(&left_msg);
				RH_pub.publish(&right_msg);
				HD_pub.publish(&hd_msg);
				Button_X_pub.publish(&Button_X_msg);
				Button_Y_pub.publish(&Button_Y_msg);
				Button_B_pub.publish(&Button_B_msg);
				Button_A_pub.publish(&Button_A_msg);
				LH_joy_button_pub.publish(&LH_Thumbstick_button_msg);
			}

			nh.spinOnce();
			//---------------------------------------------------------------------------------------------------------------end job

			now = boost::posix_time::microsec_clock::local_time();

			if (t < now)
			{
				std::cout << "task 2 !!!" << std::endl;

			}
			else
			{
				boost::posix_time::time_duration msdiff = t - now;
				boost::this_thread::sleep(msdiff);
			}

			t = t + period;
		}

	}

	flag2 = 1;
	// quit
	return 0;
}