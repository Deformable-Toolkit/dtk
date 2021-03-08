#include "dtkPhysMassSpring.h"

#ifdef DTK_DEBUG
    #define DTK_PHYSMASSSPRINGIMPL_DEBUG
#endif //DTK_DEBUG

#ifdef DTK_PHYSMASSSPRINGIMPL_DEBUG
    #include <iostream>
    using namespace std;
#endif

#include <iostream>
#include <string>
#include <fstream>
#include <set>
using namespace std;

namespace dtk
{
	
	dtkPhysMassSpring::dtkPhysMassSpring(double defaultMass,
            double defaultStiff, double defaultDamp, double defaultPointDamp, double defaultPointResistence, dtkDouble3 defaultGravityAccel )
	{
        mDefaultMass = defaultMass;
        mDefaultStiff = defaultStiff;
        mDefaultDamp = defaultDamp;
		mDefaultPointDamp = defaultPointDamp;
		mDefaultPointResistence = defaultPointResistence;
		mDefaultGravityAccel = defaultGravityAccel;
#ifdef DTK_CL        
        mUseMultiThread = false;
        source_ = "";
        mMPPosData = NULL;
        mMPVelData = NULL;
        mMPAccelData = NULL;
        mMPForceAccumData = NULL;
        mMPGravityData = NULL;
        mMPMassData = NULL;
        mMPPosBufferData = NULL;
        mMPVelBufferData = NULL;
        mMPAccelBufferData = NULL;
        mMPForceDecoratorData = NULL;
        mMPActiveData = NULL;

        mSpringVertexIDData = NULL;
        mSpringOriLengthData = NULL;
        mSpringStiffData = NULL;
        mSpringDampData = NULL;
#endif
		mUnderControl = false;
	}

	
	dtkPhysMassSpring::~dtkPhysMassSpring()
	{
        for(dtkID i = 0;i < mMassPoints.size();i++)
        {
            delete mMassPoints[i];
        }

        for(dtkID i = 0;i < mSprings.size();i++)
        {
            delete mSprings[i];
        }

	}

	
	void dtkPhysMassSpring::SetPoints(dtkPoints::Ptr points)
	{
		dtkAssert(points.get() != NULL, NULL_POINTER);
		mPts = points;
	}

	
	dtkPoints::Ptr dtkPhysMassSpring::GetPoints()
	{
		return mPts;
	}

	
	const GK::Point3& dtkPhysMassSpring::GetPoint(dtkID id) const
	{
		dtkAssert(mPts.get() != NULL, NULL_POINTER);
		return mPts->GetPoint(id);
	}

	
	void dtkPhysMassSpring::SetPoint(dtkID id, const GK::Point3 &coord)
	{
		dtkAssert(mPts.get() != NULL, ILLEGAL_STATE);
		dtkAssert(mPts->SetPoint(id, coord), OUT_OF_RANGE);
	}

	
	dtkID dtkPhysMassSpring::AddMassPoint(dtkID id, const double& mass, const dtkT3<double>& vel, double pointDamp, double pointResistence, dtkDouble3 defaultGravityAccel)
	{
		mMassPoints.push_back(new dtkPhysMassPoint(id, mPts, mass, vel, pointDamp, pointResistence, defaultGravityAccel ));

#ifdef DTK_PHYSMASSSPRINGIMPL_DEBUG
		mAltitudeSpringForces.push_back( dtkT3<double>(0,0,0) );
#endif

		return mMassPoints.size() - 1;
	}

	
	dtkID dtkPhysMassSpring::AddSpring(dtkID p1, dtkID p2, const double& stiff, const double& damp)
	{
		if( p1 > p2 )
			swap( p1, p2 );
		if( mEdgeMap.find( dtkID2( p1, p2 ) ) == mEdgeMap.end() )
		{
			dtkPhysSpring* newSpring = new dtkPhysSpring(mMassPoints[p1], mMassPoints[p2], stiff, damp);
			mSprings.push_back( newSpring );
			mEdgeMap[ dtkID2( p1, p2 ) ] = newSpring;
		}
		return mSprings.size() - 1;
	}

	
	bool dtkPhysMassSpring::Update_s(double timeslice, ItrMethod method, bool limitDeformation)
	{
        if(timeslice == 0)
            return true;
		switch(method){
		case Euler:
            PreUpdate(timeslice, method);
			UpdateStrings(timeslice, method, 0, limitDeformation);
			UpdateMassPoints(timeslice, method, 0);
            PostUpdate(method);
			break;
		case Mid:
			for(dtkID i = 0; i < 2; i++)
			{
                PreUpdate(timeslice, method, i);
				UpdateStrings(timeslice, method, i, limitDeformation);
				UpdateMassPoints(timeslice, method, i);
                PostUpdate(method, i);
			}
			break;
		case RK4:
			for(dtkID i = 0; i < 4; i++)
			{
                PreUpdate(timeslice, method, i);
				UpdateStrings(timeslice, method, i, limitDeformation);
				UpdateMassPoints(timeslice, method, i);
                PostUpdate(method, i);
			}
			break;
		case Heun:
			for(dtkID i = 0; i < 2; i++)
			{
                PreUpdate(timeslice, method, i);
#ifdef DTK_PHYSMASSSPRINGIMPL_DEBUG
                if( i == 0 )
                {
                    mAltitudeSpringForces.clear();
                    for( dtkID j = 0; j < mMassPoints.size(); j++ )
                        mAltitudeSpringForces.push_back( mMassPoints[j]->GetForceAccum() );
                }
#endif
				UpdateStrings(timeslice, method, i, limitDeformation);
				UpdateMassPoints(timeslice, method, i);
                PostUpdate(method, i);
			}
		case Collision:
			for(dtkID i = 0; i < 2; i++)
			{
                PreUpdate(timeslice, method, i);
#ifdef DTK_PHYSMASSSPRINGIMPL_DEBUG
                if( i == 0 )
                {
                    mAltitudeSpringForces.clear();
                    for( dtkID j = 0; j < mMassPoints.size(); j++ )
                        mAltitudeSpringForces.push_back( mMassPoints[j]->GetForceAccum() );
                }
#endif
				UpdateStrings(timeslice, method, i, limitDeformation);
				UpdateMassPoints(timeslice, method, i);
                PostUpdate(method, i);
            }
		}

		return true;
	}

		
	
    bool dtkPhysMassSpring::Update_iteration(double timeslice, ItrMethod method, dtkID iteration, bool limitDeformation)
    {
        if(timeslice == 0)
            return true;
        PreUpdate(timeslice, method, iteration);
#ifdef DTK_PHYSMASSSPRINGIMPL_DEBUG
                if( iteration == 0 )
                {
                    mAltitudeSpringForces.clear();
                    for( dtkID j = 0; j < mMassPoints.size(); j++ )
                        mAltitudeSpringForces.push_back( mMassPoints[j]->GetForceAccum() );
                }
#endif
		UpdateStrings(timeslice, method, iteration, limitDeformation);
		UpdateMassPoints(timeslice, method, iteration);
        PostUpdate(method, iteration);
		return true;

    }

	
    bool dtkPhysMassSpring::Update(double timeslice, ItrMethod method, bool limitDeformation)
    {
#ifdef DTK_CL
        if( mUseMultiThread )
            return Update_mt( timeslice, method, limitDeformation );
        else
#endif
            return Update_s( timeslice, method, limitDeformation );
    }

	
	bool dtkPhysMassSpring::UpdateStrings(double timeslice, ItrMethod method, dtkID iteration, bool limitDeformation)
	{
		for(dtkID i = 0; i < mSprings.size(); i++)
			mSprings[i]->Update(timeslice, method, iteration, limitDeformation);

		return true;
	}

	
	bool dtkPhysMassSpring::UpdateMassPoints(double timeslice, ItrMethod method, dtkID iteration)
	{
		for(dtkID i = 0; i < mMassPoints.size(); i++)
			mMassPoints[i]->Update(timeslice, method, iteration);

		return true;
	}

    
	bool dtkPhysMassSpring::ApplyImpulse( double timeslice )
    {
		for(dtkID i = 0; i < mMassPoints.size(); i++)
			mMassPoints[i]->ApplyImpulse();

		return true;
    }

    
	bool dtkPhysMassSpring::PreUpdate(double timeslice, ItrMethod method, dtkID iteration)
	{
		return false;
    }

    
	bool dtkPhysMassSpring::PostUpdate(ItrMethod method, dtkID iteration)
	{
		return false;
    }

	
    void dtkPhysMassSpring::SetSpringStiffness(int id, double newStiff)
    {
        if(id >= ((int)mSprings.size()) || id < 0)
            return;
        mSprings[id]->SetStiffness(newStiff);
    }

	
    void dtkPhysMassSpring::SetSpringDamp(int id, double newDamp)
    {
        if(id >= ((int)mSprings.size()) || id < 0)
            return;
        mSprings[id]->SetDamp(newDamp);
    }
        

	
    void dtkPhysMassSpring::SetPointMass(int id, double newMass)
    {
        if(id >= mMassPoints.size() || id < 0)
            return;
        mMassPoints[id]->SetMass(newMass);
    }
#ifdef DTK_CL
	bool dtkPhysMassSpring::Update_mt(double timeslice, ItrMethod method, bool limitDeformation)
	{
		if(timeslice == 0)
			return true;
		for(dtkID i = 0; i < 2; i++)
		{
			PreUpdate(timeslice, method, i);
#ifdef DTK_PHYSMASSSPRINGIMPL_DEBUG
			if( i == 0 )
			{
				mAltitudeSpringForces.clear();
				for( dtkID j = 0; j < mMassPoints.size(); j++ )
					mAltitudeSpringForces.push_back( mMassPoints[j]->GetForceAccum() );
			}
#endif
			UpdateCopyToCLArray();
			RunCLKernels(timeslice, i);
			UpdateCopyBack();

			PostUpdate(method, i);
		}
		return true;
	}

    template<typename double>
    int dtkPhysMassSpring::SetupCL()
    {
        cl_int status;
        clGetPlatformIDs(1, &platform, NULL);
        clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);
        context = clCreateContext( NULL, 
                                    1,
                                    &device,
                                    NULL, NULL, NULL);
        queue = clCreateCommandQueue( context,
                                        device,
                                        0, &status );
        if(status != CL_SUCCESS)
        {
            cout<<"create command error!"<<endl;
        }
        const char *source = source_.c_str();
        size_t sourceSize[] = {strlen(source_.c_str())};
        program = clCreateProgramWithSource(context,
                                            1,
                                            &source,
                                            sourceSize,
                                            &status);
        if(status != CL_SUCCESS)
        {
            cout<<"create program with source error!"<<endl;
        }
        const char* option = "-g -O0";
        status = clBuildProgram( program, 1, &device, option, NULL, NULL);
        if(status != CL_SUCCESS)
        {
            cout<<"build program error!"<<endl;
            if(status == CL_INVALID_PROGRAM)
                cout<<"invalid program"<<endl;
            if(status == CL_INVALID_VALUE)
                cout<<"invalid value"<<endl;
            if(status == CL_INVALID_DEVICE)
                cout<<"invalid device"<<endl;
            if(status == CL_INVALID_BUILD_OPTIONS)
                cout<<"invalid build options"<<endl;
            if(status == CL_INVALID_OPERATION)
                cout<<"invalid operation"<<endl;
            if(status == CL_COMPILER_NOT_AVAILABLE)
                cout<<"compiler not available"<<endl;
            if(status == CL_BUILD_PROGRAM_FAILURE)
                cout<<"build program failure"<<endl;
        }
        kernelSpring = clCreateKernel( program, "dtkPhysSpringUpdate", NULL);
        kernelMP = clCreateKernel( program, "dtkPhysPointUpdate", NULL);
                                 
        mSpringOriLengthBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * sizeof(cl_float),
                                        mSpringOriLengthData,
                                        0);
        mSpringStiffBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * sizeof(cl_float),
                                        mSpringStiffData,
                                        0);
        mSpringDampBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * sizeof(cl_float),
                                        mSpringDampData,
                                        0);

        mSpringVertexIDBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * 2.0 * sizeof(cl_int),
                                        mSpringVertexIDData,
                                        0);
        mMPPosBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 3.0 * sizeof(cl_float),
                                        mMPPosData,
                                        0);


        mMPVelBuffer = clCreateBuffer(context, 
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 3 * sizeof(cl_float),
                                        mMPVelData,
                                        0);

        mMPAccelBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 3 * sizeof(cl_float),
                                        mMPAccelData,
                                        0);
        mMPForceAccumBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 3 * sizeof(cl_float),
                                        mMPForceAccumData,
                                        0);
        mMPGravityBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 3 * sizeof(cl_float),
                                        mMPGravityData,
                                        0);
        mMPMassBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * sizeof(cl_float),
                                        mMPMassData,
                                        &status);
        this->CheckCLStatus(status, "massbuffer create buffer error!");
        mMPPosBufferBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 9 * sizeof(cl_float),
                                        mMPPosBufferData,
                                        0);
        mMPVelBufferBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 9 * sizeof(cl_float),
                                        mMPVelBufferData,
                                        0);
        mMPAccelBufferBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 9 * sizeof(cl_float),
                                        mMPAccelBufferData,
                                        0);
        mMPForceDecoratorBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * 3 * sizeof(cl_float),
                                        mMPForceDecoratorData,
                                        0);
        mMPActiveBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * sizeof(cl_bool),
                                        mMPActiveData,
                                        &status);
        mMPAdjacentVertexIDsBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * sizeof(cl_int) * 2,
                                        mMPAdjacentVertexIDsData,
                                        &status);
        mMPNumberOfAdjacentVertexBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfMassPoints() * sizeof(cl_int),
                                        mMPNumberOfAdjacentVertexData,
                                        &status);
        this->CheckCLStatus(status, "numofadjvertexbuffer create buffer error!");
        mMPStartAdjacentVertexIDBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * sizeof(cl_int),
                                        mMPStartAdjacentVertexIDData,
                                        &status);
        mMPAdjacentSpringIDsBuffer = clCreateBuffer(context,
                                        CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                        this->GetNumberOfSprings() * sizeof(cl_int) * 2,
                                        mMPAdjacentSpringIDsData,
                                        &status);
        global_work_size = NWITEMS;
        mUseMultiThread = true;
    }

    template<typename double>
    int dtkPhysMassSpring::RunCLKernels(double timeslice, dtkID iteration)
    {
        mTimesliceData = static_cast<cl_float>( timeslice );
        mIterationData = iteration; 
        cl_int status;
       
        status = clSetKernelArg(kernelMP, 0, sizeof(cl_int), (void*) &mNumberOfMassPoints);
        //this->CheckCLStatus(status, "set kernel arg error! MP 0");
        status = clSetKernelArg(kernelMP, 1, sizeof(cl_mem), (void*) &mMPPosBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 1");
        status = clSetKernelArg(kernelMP, 2, sizeof(cl_mem), (void*) &mMPVelBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 2");
        status = clSetKernelArg(kernelMP, 3, sizeof(cl_mem), (void*) &mMPAccelBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 3");
        status = clSetKernelArg(kernelMP, 4, sizeof(cl_mem), (void*) &mMPForceAccumBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 4");
        status = clSetKernelArg(kernelMP, 5, sizeof(cl_mem), (void*) &mMPGravityBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 5");
        status = clSetKernelArg(kernelMP, 6, sizeof(cl_mem), (void*) &mMPMassBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 6");
        status = clSetKernelArg(kernelMP, 7, sizeof(cl_mem), (void*) &mMPPosBufferBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 7");
        status = clSetKernelArg(kernelMP, 8, sizeof(cl_mem), (void*) &mMPVelBufferBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 8");
        status = clSetKernelArg(kernelMP, 9, sizeof(cl_mem), (void*) &mMPAccelBufferBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 9");
        status = clSetKernelArg(kernelMP, 10, sizeof(cl_mem), (void*) &mMPForceDecoratorBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 10");
        status = clSetKernelArg(kernelMP, 11, sizeof(cl_mem), (void*) &mMPActiveBuffer);
        //this->CheckCLStatus(status, "set kernel arg error! MP 11");
        status = clSetKernelArg(kernelMP, 12, sizeof(cl_int), &mIterationData);
        //this->CheckCLStatus(status, "set kernel arg error! MP 12");
        status = clSetKernelArg(kernelMP, 13, sizeof(cl_float), &mTimesliceData);
        //this->CheckCLStatus(status, "set kernel arg error! MP 13");
        status = clSetKernelArg(kernelMP, 14, sizeof(cl_mem), (void*) &mMPAdjacentVertexIDsBuffer);
        status = clSetKernelArg(kernelMP, 15, sizeof(cl_mem), (void*) &mMPNumberOfAdjacentVertexBuffer);
        status = clSetKernelArg(kernelMP, 16, sizeof(cl_mem), (void*) &mMPStartAdjacentVertexIDBuffer);
        status = clSetKernelArg(kernelMP, 17, sizeof(cl_mem), (void*) &mMPAdjacentSpringIDsBuffer);
        status = clSetKernelArg(kernelMP, 18, sizeof(cl_mem), (void*) &mSpringOriLengthBuffer);
        status = clSetKernelArg(kernelMP, 19, sizeof(cl_mem), (void*) &mSpringStiffBuffer);
        status = clSetKernelArg(kernelMP, 20, sizeof(cl_mem), (void*) &mSpringDampBuffer);
        
        work_size = this->GetNumberOfMassPoints();
        size_t local_work_size = 256;
        work_size = (work_size / local_work_size + 1) * local_work_size;
        clEnqueueNDRangeKernel( queue,
                                kernelMP,
                                1,
                                NULL,
                                &work_size,
                                &local_work_size, 0, NULL, NULL);
        //clFinish( queue );
        
        clEnqueueReadBuffer(queue, mMPPosBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 3 * sizeof(cl_float), mMPPosData, 0, NULL, NULL); //
        clEnqueueReadBuffer(queue, mMPVelBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 3 * sizeof(cl_float), mMPVelData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPPosBufferBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 9 * sizeof(cl_float), mMPPosBufferData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPVelBufferBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 9 * sizeof(cl_float), mMPVelBufferData, 0, NULL, NULL ); //
        clFinish(queue);
        
        /*
        clEnqueueReadBuffer(queue, mMPAccelBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 3.0 * sizeof(cl_float), mMPAccelData, 0, NULL, NULL );
        clEnqueueReadBuffer(queue, mMPForceAccumBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 3.0 * sizeof(cl_float), mMPForceAccumData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPGravityBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 3.0 * sizeof(cl_float), mMPGravityData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPMassBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * sizeof(cl_float), mMPMassData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPAccelBufferBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 9.0 * sizeof(cl_float), mMPAccelBufferData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPForceDecoratorBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * 3.0 * sizeof(cl_float), mMPForceDecoratorData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mMPActiveBuffer, CL_TRUE, 0, 
                this->GetNumberOfMassPoints() * sizeof(cl_bool), mMPActiveData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mSpringVertexIDBuffer, CL_TRUE, 0, 
                this->GetNumberOfSprings() * 2.0 * sizeof(cl_int), mSpringVertexIDData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mSpringOriLengthBuffer, CL_TRUE, 0, 
                this->GetNumberOfSprings() * sizeof(cl_float), mSpringOriLengthData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mSpringStiffBuffer, CL_TRUE, 0, 
                this->GetNumberOfSprings() * sizeof(cl_float), mSpringStiffData, 0, NULL, NULL ); //
        clEnqueueReadBuffer(queue, mSpringDampBuffer, CL_TRUE, 0, 
                this->GetNumberOfSprings() * sizeof(cl_float), mSpringDampData, 0, NULL, NULL ); //
                */
        /*
        cout << "RunKernel Ouptput:" << endl;
        cout << mSpringOriLengthData[0] << endl;
        cout << mSpringStiffData[0] << endl;
        cout << mSpringDampData[0] << endl;
        cout << mSpringVertexIDData[0] << endl;
        cout << mMPPosData[0] << endl;
        cout << mMPVelData[0] << endl;
        cout << mMPForceAccumData[0] << endl;
        cout << mMPGravityData[0] << endl;
        cout << mMPMassData[0] << endl;
        cout << mMPPosBufferData[0] << endl;
        cout << mMPVelBufferData[0] << endl;
        cout << mMPAccelBufferData[0] << endl;
        cout << mMPForceDecoratorData[0] << endl;
        cout << mMPActiveData[0] << endl;
        cout << mMPAccelData[0] << endl;
        cout << "/RunKernel Ouptput" << endl;
        */

		return 0;
    }

    
    bool dtkPhysMassSpring::Open(const char* fileName)
    {
        size_t size;
        char* str;

        fstream f(fileName);//, (fstream::in | fstream::binary));
        if(!f)
            return false;
        else//if(f.is_open())
        {
            size_t sizeFile;
            f.seekg(0, fstream::end);
            size = sizeFile = (size_t)f.tellg();
            f.seekg(0, fstream::beg);

            str = new char[size + 1];
            if(!str)
            {
                f.close();
                return false;
            }

            f.read(str, sizeFile);
            f.close();
            str[size] = '\0';

            source_ = str;
            delete[] str;
            return true;
        }
    }
   
    
    void dtkPhysMassSpring::InitialCLArray()
    {
        size_t sizeInBytes = this->GetNumberOfMassPoints() * sizeof(cl_float);
        mMPPosData = (cl_float*) malloc(sizeInBytes * 3.0);
        mMPVelData = (cl_float*) malloc(sizeInBytes * 3.0);
        mMPAccelData = (cl_float*) malloc(sizeInBytes * 3.0);
        mMPForceAccumData = (cl_float*) malloc(sizeInBytes * 3.0);
        mMPForceDecoratorData = (cl_float*) malloc(sizeInBytes * 3.0);
        mMPGravityData = (cl_float*) malloc(sizeInBytes * 3.0);
        mMPMassData = (cl_float*) malloc(sizeInBytes);
        mMPPosBufferData = (cl_float*) malloc(sizeInBytes * 9.0);
        mMPVelBufferData = (cl_float*) malloc(sizeInBytes * 9.0);
        mMPAccelBufferData = (cl_float*) malloc(sizeInBytes * 9.0);

        sizeInBytes = this->GetNumberOfMassPoints() * sizeof(cl_bool);
        mMPActiveData = (cl_bool*) malloc(sizeInBytes);

        sizeInBytes = this->GetNumberOfSprings() * sizeof(cl_int);
        mSpringVertexIDData = (cl_int*) malloc(sizeInBytes * 2.0); 

        sizeInBytes = this->GetNumberOfSprings() * sizeof(cl_float);
        mSpringOriLengthData = (cl_float*) malloc(sizeInBytes);
        mSpringStiffData = (cl_float*) malloc(sizeInBytes);
        mSpringDampData = (cl_float*) malloc(sizeInBytes);

        sizeInBytes = this->GetNumberOfSprings() * 2.0 * sizeof(cl_int);
        mMPAdjacentVertexIDsData = (cl_int*) malloc(sizeInBytes);
        mMPAdjacentSpringIDsData = (cl_int*) malloc(sizeInBytes);
        sizeInBytes = this->GetNumberOfMassPoints() * sizeof(cl_int);
        mMPNumberOfAdjacentVertexData = (cl_int*) malloc(sizeInBytes);
        mMPStartAdjacentVertexIDData = (cl_int*) malloc(sizeInBytes);

        vector< vector<int> > tmpAdjacentVertexIDs;
        vector< vector<int> > tmpAdjacentSpringIDs;
        tmpAdjacentVertexIDs.resize( this->GetNumberOfMassPoints() );
        tmpAdjacentSpringIDs.resize( this->GetNumberOfMassPoints() );
        
        for(dtkID i = 0; i < mSprings.size(); i++)
        {
            int id1,id2;
            id1 = mSpringVertexIDData[i * 2] = mSprings[i]->GetFirstVertex()->GetPointID();
            id2 = mSpringVertexIDData[i * 2 + 1] = mSprings[i]->GetSecondVertex()->GetPointID(); 

            tmpAdjacentVertexIDs[id1].push_back(mSprings[i]->GetSecondVertex()->GetPointID());
            tmpAdjacentVertexIDs[id2].push_back(mSprings[i]->GetFirstVertex()->GetPointID());
            tmpAdjacentSpringIDs[id1].push_back(i);
            tmpAdjacentSpringIDs[id2].push_back(i);

            mSpringOriLengthData[i] = mSprings[i]->GetOriLength();
            mSpringStiffData[i] = mSprings[i]->GetStiffness();
            mSpringDampData[i] = mSprings[i]->GetDamp();
        }

        int startADJ = 0;
        int startSADJ = 0;
		for(dtkID i = 0; i < mMassPoints.size(); i++)
        {
            dtkT3<double> pos = mMassPoints[i]->GetPosition();
            dtkT3<double> vel = mMassPoints[i]->GetVel();
            dtkT3<double> accel = mMassPoints[i]->GetAccel();
            dtkT3<double> forceAccum = mMassPoints[i]->GetForceAccum();
            dtkT3<double> gravity = mMassPoints[i]->GetGravity();
            dtkT3<double> forceDecorator = mMassPoints[i]->GetForceDecorator();
            std::vector< dtkT3<double> > posBuffer;
            posBuffer.resize(3);
            posBuffer = mMassPoints[i]->GetPosBuffer();
            std::vector< dtkT3<double> > velBuffer;
            velBuffer.resize(3);
            velBuffer = mMassPoints[i]->GetVelBuffer();
            std::vector< dtkT3<double> > accelBuffer;
            accelBuffer.resize(3);
            accelBuffer = mMassPoints[i]->GetAccelBuffer();
            bool activity = mMassPoints[i]->GetActive();
            mMPMassData[i] = mMassPoints[i]->GetMass();
            mMPActiveData[i] = activity;
            for(int j = 0;j < 3;j++)
            {
                mMPPosData[i * 3 + j] = pos[j];
                mMPVelData[i * 3 + j] = vel[j];
                mMPAccelData[i * 3 + j] = accel[j];
                mMPForceAccumData[i * 3 + j] = forceAccum[j];
                mMPGravityData[i * 3 + j] = gravity[j];
                mMPForceDecoratorData[i * 3 + j] = forceDecorator[j];
            }
            for(int j = 0;j < 3;j++)
            {
                for(int m = 0;m < 3;m++)
                {
                    mMPPosBufferData[i * 9 + j * 3 + m] = posBuffer[j][m];
                    mMPVelBufferData[i * 9 + j * 3 + m] = velBuffer[j][m];
                    mMPAccelBufferData[i * 9 + j * 3 + m] = accelBuffer[j][m];
                }
            }

            mMPStartAdjacentVertexIDData[i] = startADJ;
            mMPNumberOfAdjacentVertexData[i] = tmpAdjacentVertexIDs[i].size();
            for(int j = 0; j < tmpAdjacentVertexIDs[i].size(); j++)
                mMPAdjacentVertexIDsData[startADJ++] = tmpAdjacentVertexIDs[i][j];
            for(int j = 0; j < tmpAdjacentSpringIDs[i].size(); j++)
                mMPAdjacentSpringIDsData[startSADJ++] = tmpAdjacentSpringIDs[i][j];
        }

        assert(startADJ == this->GetNumberOfSprings() * 2);
        assert(startSADJ == this->GetNumberOfSprings() * 2);

        mNumberOfMassPoints = this->GetNumberOfMassPoints();
        /*
        mSpringOriLengthData[0] = 1;
        mSpringStiffData[0] = 2;
        mSpringDampData[0] = 3;
        mSpringVertexIDData[0] = 4;
        mMPPosData[0] = 5;
        mMPVelData[0] = 6;
        mMPForceAccumData[0] = 7;
        mMPGravityData[0] = 8;
        mMPMassData[0] = 9;
        mMPPosBufferData[0] = 10;
        mMPVelBufferData[0] = 11;
        mMPAccelBufferData[0] = 12;
        mMPForceDecoratorData[0] = 13;
        mMPActiveData[0] = true;
        mMPAccelData[0] = 15;
        */
    }

    
    void dtkPhysMassSpring::UpdateCopyToCLArray()
    {
		for(dtkID i = 0; i < mMassPoints.size(); i++)
        {
            //dtkT3<T> vel = mMassPoints[i]->GetVel();
            //dtkT3<T> accel = mMassPoints[i]->GetAccel();
            //dtkT3<T> pos = mMassPoints[i]->GetPosition();
            dtkT3<double> forceAccum = mMassPoints[i]->GetForceAccum();
            /*
            dtkT3<T> forceDecorator = mMassPoints[i]->GetForceDecorator();
            vector< dtkT3<T> > posBuffer = mMassPoints[i]->GetPosBuffer();
            vector< dtkT3<T> > velBuffer = mMassPoints[i]->GetVelBuffer();
            vector< dtkT3<T> > accelBuffer = mMassPoints[i]->GetAccelBuffer();
            */
            bool activity = mMassPoints[i]->IsActive();
            mMPActiveData[i] = activity;
            for(int j = 0;j < 3;j++)
            {
                //mMPVelData[i * 3 + j] = vel[j];
                //mMPAccelData[i * 3 + j] = accel[j];
                //mMPPosData[i * 3 + j] = pos[j];
                mMPForceAccumData[i * 3 + j] = static_cast<cl_float>( forceAccum[j] );
            }
            /*
            for(int j = 0;j < 4;j++)
            {
                for(int m = 0;m < 3;m++)
                {
                    mMPPosBufferData[i * 12 + j * 3 + m] = posBuffer[j][m];
                    mMPVelBufferData[i * 12 + j * 3 + m] = velBuffer[j][m];
                    mMPAccelBufferData[i * 12 + j * 3 + m] = accelBuffer[j][m];
                }
            }
            */
        }
		/*
        cl_int status;
        status = clEnqueueWriteBuffer(queue,
                            mMPPosBuffer,
                            CL_TRUE,
                            0,
                            sizeof(cl_float) * this->GetNumberOfMassPoints() * 3.0,
                            mMPPosData,
                            0, NULL, NULL);

        status = clEnqueueWriteBuffer(queue,
                            mMPActiveBuffer,
                            CL_TRUE,
                            0,
                            sizeof(cl_bool) * this->GetNumberOfMassPoints(),
                            mMPActiveData,
                            0, NULL, &writeEvt);
        this->CheckCLStatus( status, "write to spring vertex buffer error!");
        status = WaitForEventAndRelease(&writeEvt);
        this->CheckCLStatus( status, "wait for evt and release error!");
        */
        clEnqueueWriteBuffer(queue,
                            mMPForceAccumBuffer,
                            CL_TRUE,
                            0,
                            sizeof(cl_float) * this->GetNumberOfMassPoints() * 3,
                            mMPForceAccumData,
                            0, NULL, NULL);
        //clFinish(queue);
    }
    
    void dtkPhysMassSpring::UpdateCopyBack()
    {
		for(dtkID i = 0; i < mMassPoints.size(); i++)
        {
            dtkT3<double> vel(mMPVelData[i * 3], mMPVelData[i * 3 + 1], mMPVelData[i * 3 + 2]);
            dtkT3<double> pos(mMPPosData[i * 3], mMPPosData[i * 3 + 1], mMPPosData[i * 3 + 2]);
            mMassPoints[i]->SetPosition(pos);
            mMassPoints[i]->SetVel(vel);
            dtkT3<double> velBuffer, posBuffer;
            for(int m = 0;m < 3;m++)
            {
                velBuffer[m] = mMPVelBufferData[i * 9 + m];
                posBuffer[m] = mMPPosBufferData[i * 9 + m];
            }
            mMassPoints[i]->SetVel(velBuffer,0);
            mMassPoints[i]->SetPosition(posBuffer,0);
            mMassPoints[i]->SetForceAccum(dtkT3<double>(0,0,0));
        }
    }

    
    void dtkPhysMassSpring::CheckCLStatus(cl_int status, string message)
    {
        if(status != CL_SUCCESS)
            cout<<message<<endl;
        return;
    }
 
    
    int dtkPhysMassSpring::WaitForEventAndRelease(cl_event *event)
    {
        cl_int status = CL_SUCCESS;
        cl_int eventStatus = CL_QUEUED;
        while(eventStatus != CL_COMPLETE)
        {
            status = clGetEventInfo(
                            *event,
                            CL_EVENT_COMMAND_EXECUTION_STATUS,
                            sizeof(cl_int),
                            &eventStatus,
                            NULL);
            CheckCLStatus(status, "clGetEventInfo error!");
        }
        status = clReleaseEvent(*event);
        CheckCLStatus(status, "clReleaseEvent error!");
        return CL_SUCCESS;
    }
                            
    
    void dtkPhysMassSpring::UseMultiThread(const char* fileName)
    {
        this->Open(fileName);
        this->InitialCLArray();
        this->SetupCL();
    }
#endif 
    
    size_t dtkPhysMassSpring::FindTwins( Ptr ms, double distance)
    {
		size_t count = 0;

        for( dtkID i = 0; i < this->GetNumberOfMassPoints(); i++ )
        {
            dtkPhysMassPoint* point1 = this->GetMassPoint( i );
			if( point1->HasTwin() )
				continue;

            double minDistance = dtkDoubleMax;
            dtkT3<double> pos1 = point1->GetPosition();
			            
			dtkPhysMassPoint* point2;
            dtkT3<double> pos2;
            dtkID minDistanceID = 0;

            for( dtkID j = 0; j < ms->GetNumberOfMassPoints(); j++ )
            {
				point2 = ms->GetMassPoint( j );
                if( point2->HasTwin() )
                    continue;
                double tempDistance;
                pos2 = point2->GetPosition();
                tempDistance = length( pos1 - pos2 );

                if( tempDistance < minDistance )
                {
                    minDistance = tempDistance;
                    minDistanceID = j;
                }
            }
			if( minDistance < distance)
			{
				point1->AddTwin( ms->GetMassPoint( minDistanceID ) );
				ms->GetMassPoint( minDistanceID )->AddTwin( point1 );
				count++;
			}
        }

		return count;
    }

	void dtkPhysMassSpring::ConvertImpulseToForce( double timeslice )
	{
		mImpulseForce = dtkT3<double>( 0, 0, 0 );
		for( dtkID i = 0; i < this->GetNumberOfMassPoints(); i++ )
		{
			dtkPhysMassPoint* point = this->GetMassPoint( i );
			mImpulseForce = mImpulseForce + point->GetAndClearImpulse();
		}
		mImpulseForce = mImpulseForce * ( 1.0 / timeslice );
	}

	void dtkPhysMassSpring::RegisterLabel( dtkID label )
	{
		mLabels.push_back( label );
		mTransportForces[ label ] = dtkT3<double>( 0,0,0 );
	}

	void dtkPhysMassSpring::TransportForce( double timeslice )
	{
		for( std::map< dtkID, dtkT3<double> >::iterator itr = mTransportForces.begin();
			itr != mTransportForces.end(); itr++ )
		{
			mTransportForces[ itr->first ] = dtkT3<double>( 0,0,0 );
		}
		for( dtkID i = 0; i < this->GetNumberOfMassPoints(); i++ )
		{
			dtkPhysMassPoint* point = this->GetMassPoint( i );
			if( !point->IsActive() )
			{
				dtkID label = point->GetLabel();
				if( label != 0 )
				{
					mTransportForces[label] = mTransportForces[label] + point->GetForceAccum();
				}
			}
		}
	}
	
	void dtkPhysMassSpring::SetTriangleMesh( dtkStaticTriangleMesh::Ptr newTriangleMesh )
	{
		this->SetPoints( newTriangleMesh->GetPoints() );
		for( dtkID i = 0; i < newTriangleMesh->GetNumberOfPoints(); i++ )
		{
			this->AddMassPoint( i, mDefaultMass, dtkT3<double>(0,0,0), mDefaultPointDamp, mDefaultPointResistence, mDefaultGravityAccel );
		}
		const std::vector<dtkID3>& ec = newTriangleMesh->GetECTable();
		set<dtkID2> edges;
		for( dtkID i = 0; i < ec.size(); i++ )
		{
			if( ec[i][0] > ec[i][1] )
				edges.insert(dtkID2( ec[i][1], ec[i][0] ));
			else
				edges.insert(dtkID2( ec[i][0], ec[i][1] ));

			if( ec[i][0] > ec[i][2] )
				edges.insert(dtkID2( ec[i][2], ec[i][0] ));
			else
				edges.insert(dtkID2( ec[i][0], ec[i][2] ));

			if( ec[i][2] > ec[i][1] )
				edges.insert(dtkID2( ec[i][1], ec[i][2] ));
			else
				edges.insert(dtkID2( ec[i][2], ec[i][1] ));
		}

		for( set<dtkID2>::iterator itr = edges.begin(); itr != edges.end(); itr++ )
		{
			this->AddSpring( itr->a, itr->b, mDefaultStiff, mDefaultDamp );
		}
	}


	dtkPhysSpring* dtkPhysMassSpring::GetSpringByPoints(dtkID2 edge)
	{
		map< dtkID2, dtkPhysSpring* >::iterator it;
		it = mEdgeMap.find(edge);
		if(it != mEdgeMap.end())
			return it->second;
		else
			return 0;
	}


	void dtkPhysMassSpring::DeleteSpring( dtkID id1, dtkID id2 )
	{
		if( id1 > id2 )
			swap( id1, id2 );
		dtkPhysSpring* spring = mEdgeMap[ dtkID2( id1, id2 ) ];
		for( dtkID i = 0; i < mSprings.size(); i++ )
		{
			if( mSprings[i] == spring )
			{
				mSprings.erase( mSprings.begin() + i );
				break;
			}
		}
		delete spring;
	}


	void dtkPhysMassSpring::AbandonTwins()
	{
		for( dtkID i = 0; i < GetNumberOfMassPoints(); i++ )
		{
			mMassPoints[i]->AbandonTwins();
		}
	}
}
