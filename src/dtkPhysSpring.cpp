#include "dtkPhysSpring.h"
#ifdef DTK_DEBUG
    #define DTK_PHYSSPRINGIMPL_DEBUG
#endif

#ifdef DTK_PHYSSPRINGIMPL_DEBUG
    #include <iostream>
    using namespace std;
#endif // DTK_PHYSSPRINGIMPL_DEBUG

namespace dtk
{
	
	dtkPhysSpring::dtkPhysSpring(dtkPhysMassPoint* p1, 
		dtkPhysMassPoint* p2, const double& stiff, const double& damp)
	{
		mVerteces[0] = p1;
		mVerteces[1] = p2;

		mStiffness = stiff;
		mDamp = damp;
		GK::Vector3 vec = mVerteces[1]->GetPoint() - mVerteces[0]->GetPoint();
		mOriLength = length(dtkT3<double>(vec.x(), vec.y(), vec.z()));

	}

	
	dtkPhysSpring::~dtkPhysSpring()
	{

	}

	
	bool dtkPhysSpring::Update(double timeslice, ItrMethod method, dtkID iteration, bool limitDeformation)
	{
        dtkT3<double> vec = mVerteces[1]->GetPosition(method, iteration) - mVerteces[0]->GetPosition(method, iteration);
        dtkT3<double> dir = normalize(vec);
        dtkT3<double> v0 = mVerteces[0]->GetVel(method, iteration);
        dtkT3<double> v1 = mVerteces[1]->GetVel(method, iteration);
        double curLength = length(vec);
        dtkT3<double> pos1 = mVerteces[0]->GetPosition(method, 0);
        dtkT3<double> pos2 = mVerteces[1]->GetPosition(method, 0);

        dtkT3<double> tempVec = pos1 - pos2;
        if(limitDeformation && method == Heun && iteration == 1)
        {
            if(curLength > 1.05 * length(tempVec))
            {
                /*
                T t;
                tempVec = normalize(tempVec);
                dtkT3<T> v = v0 - v1;
                v = tempVec * dot(v,tempVec);
                t = (0.005 *  length(pos1 - pos2)) / timeslice; 
#ifdef DTK_PHYSSPRINGIMPL_DEBUG
                cout<<"pos1: "<<pos1[0]<<" "<<pos1[1]<<" "<<pos1[2]<<endl;
                cout<<"pos2: "<<pos2[0]<<" "<<pos2[1]<<" "<<pos2[2]<<endl;
                cout<<"oriLength "<<mOriLength<<endl;

                cout<<"length(pos1 - pos2) "<<length(pos1 - pos2)<<endl;
                cout<<"timeslice: "<<timeslice<<endl;
                cout<<"t: "<<t<<endl;
#endif // DTK_PHYSSPRINGIMPL_DEBUG
                v = v - normalize(v) * t;
                mVerteces[0]->SetVel(v0 - v * 0.5,0);
                mVerteces[1]->SetVel(v1 + v * 0.5,0);
#ifdef DTK_PHYSSPRINGIMPL_DEBUG
                cout<<"v0 after modification: "<<(v0-v * 0.5)[0]<<" "<<(v0-v * 0.5)[1]<<" "<<(v0-v * 0.5)[2]<<endl;
                cout<<"v1 after modification: "<<(v1-v * 0.5)[0]<<" "<<(v1-v * 0.5)[1]<<" "<<(v1-v * 0.5)[2]<<endl;
#endif // DTK_PHYSSPRINGIMPL_DEBUG
*/
                
                /*
                dtkT3<T> u1 = normalize(tempVec);//v0 * dot(normalize(v0),tempVec);
                dtkT3<T> u2 = -u1;//v1 * dot(normalize(v1),-tempVec);
                dtkT3<T> vec1 = pos1 - pos2 + (v0 - v1) * timeslice;
                dtkT3<T> vec2 = (u2 - u1) * timeslice;
                T t3 = dot(vec1,vec2);
                T t1 = ((-2.0) * t3 + sqrt(4.0 * t3 * t3 - 4.0 * dot(vec2,vec2) * (dot(vec1,vec1) - 1.05 * 1.05 * mOriLength * mOriLength))) / (2.0 * dot(vec2,vec2));
                T t2 = ((-2.0) * t3 - sqrt(4.0 * t3 * t3 - 4.0 * dot(vec2,vec2) * (dot(vec1,vec1) - 1.05 * 1.05 * mOriLength * mOriLength))) / (2.0 * dot(vec2,vec2));
                if(t2 > 0)
                    t = t2;
                else
                    t = t1;
                cout<<"v0: "<<v0[0]<<" "<<v0[1]<<" "<<v0[2]<<endl;
                cout<<"v1: "<<v1[0]<<" "<<v1[1]<<" "<<v1[2]<<endl;
                cout<<"u1: "<<u1[0]<<" "<<u1[1]<<" "<<u1[2]<<endl;
                cout<<"u2: "<<u2[0]<<" "<<u2[1]<<" "<<u2[2]<<endl;
                cout<<"t: "<<t<<endl;
                cout<<"vec2: "<<vec2[0]<<" "<<vec2[1]<<" "<<vec2[2]<<endl;
                cout<<"timeslice: "<<timeslice<<endl;


                mVerteces[0]->SetVel(v0 - u1 * t,0);
                mVerteces[1]->SetVel(v1 - u2 * t,0);
                cout<<"v0 after modification: "<<(v0-u1 * t)[0]<<" "<<(v0-u1 * t)[1]<<" "<<(v0-u1 * t)[2]<<endl;
                cout<<"v1 after modification: "<<(v1-u2 * t)[0]<<" "<<(v1-u2 * t)[1]<<" "<<(v1-u2 * t)[2]<<endl;
                */
            }

            /*
            if(curLength < 0.95 * mOriLength)
            {
                T t;
                dtkT3<T> pos1 = mVerteces[0]->GetPosition(method, 0);
                dtkT3<T> pos2 = mVerteces[1]->GetPosition(method, 0);
                dtkT3<T> tempVec = pos1 - pos2;
                tempVec = normalize(tempVec);
                dtkT3<T> u1 = v0 * dot(normalize(v0),tempVec);
                u1 = normalize(u1);
                dtkT3<T> u2 = v1 * dot(normalize(v1),-tempVec);
                u2 = normalize(u2);
                dtkT3<T> vec1 = pos1 - pos2 + (v0 - v1) * timeslice;
                dtkT3<T> vec2 = (u2 - u1) * timeslice;
                T t3 = dot(vec1,vec2);
                T t1 = ((-2.0) * t3 + sqrt(4.0 * t3 * t3 - 4.0 * dot(vec2,vec2) * (dot(vec1,vec1) - 0.95 * 0.95 * mOriLength * mOriLength))) / (2.0 * dot(vec2,vec2));
                T t2 = ((-2.0) * t3 - sqrt(4.0 * t3 * t3 - 4.0 * dot(vec2,vec2) * (dot(vec1,vec1) - 0.95 * 0.95 * mOriLength * mOriLength))) / (2.0 * dot(vec2,vec2));
                if(t2 > 0)
                    t = t2;
                else
                    t = t1;
                mVerteces[0]->SetVel(v0 - u1 * t,0);
                mVerteces[1]->SetVel(v1 - u2 * t,0);
                cout<<"v0: "<<v0[0]<<" "<<v0[1]<<" "<<v0[2]<<endl;
                cout<<"v1: "<<v1[0]<<" "<<v1[1]<<" "<<v1[2]<<endl;

                cout<<"v0 after modification: "<<(v0-u1 * t)[0]<<" "<<(v0-u1 * t)[1]<<" "<<(v0-u1 * t)[2]<<endl;
                cout<<"v1 after modification: "<<(v1-u2 * t)[0]<<" "<<(v1-u2 * t)[1]<<" "<<(v1-u2 * t)[2]<<endl;
            
            }
            */
        }
		dtkT3<double> stiffForce = dir * ((mOriLength - curLength) * mStiffness / mOriLength);
		dtkT3<double> dampForce = - dir * ( dot( (v1 - v0), dir ) * (timeslice * mStiffness / mOriLength + mDamp) );
		dtkT3<double> force = stiffForce + dampForce;
		mVerteces[0]->AddForce(force*(-1.0));
		mVerteces[1]->AddForce(force);
		return true;
	}
}
