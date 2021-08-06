#ifndef dtkPHYSMASSPOINT_H
#define dtkPHYSMASSPOINT_H

#include "dtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkTx.h"
#include "dtkPoints.h"


namespace dtk
{
	// 迭代方法分别是 
	//1.欧拉， （一阶）
	//2.中点法， https://en.wikipedia.org/wiki/Midpoint_method （二阶）
	//3.Runge-Kutta Families(RK4) （高阶）
	//4. Heun https://zh.wikipedia.org/wiki/Heun%E6%96%B9%E6%B3%95 （二阶）
	//5.
	//6.Position-Based / Verlet Integration https://en.wikipedia.org/wiki/Verlet_integration （二阶）
	enum ItrMethod {Euler,  /**< 欧拉 （一阶）*/
					Mid, /**< 中点法， https://en.wikipedia.org/wiki/Midpoint_method （二阶）*/
					RK4, /**< Runge-Kutta Families(RK4) （四阶）*/
					Heun, /**< Heun https://zh.wikipedia.org/wiki/Heun%E6%96%B9%E6%B3%95 （二阶）*/
					Collision, /**< */
					Verlet /**< Position-Based / Verlet Integration https://en.wikipedia.org/wiki/Verlet_integration （二阶）*/
					};

	/**
	* @class <dtkPhysMassPoint> 
	* @brief 物理弹性质点
	* @author <>
	* @note
	* 物理弹性质点
	*/
	class dtkPhysMassPoint
	{
	public:
		dtkPhysMassPoint(dtkID id, dtkPoints::Ptr pts, const double& mass = 1.0, const dtkT3<double>& vel = dtkT3<double>(0,0,0), double pointDamp = 1.0, double pointResistence = 2.5, dtkDouble3 gravityAccel = dtkT3<double>( 0,0,0 ) );

	public:
		virtual ~dtkPhysMassPoint();

		/**
		* @brief		更新质点位置，速度，加速度等
		* @param[in]	times;ice : 时间间隔
		* @param[in]	method : 迭代算法	
		* @param[in]	iteration : 迭代次数
		* @note	使用不同的算法迭代更新质点位置。
		* @return		
		*	true update successfully \n
		*	false update failure \n
		*/				
		bool Update(double timeslice, ItrMethod method = Euler, dtkID iteration = 0);

		/**
		* @brief		传递冲量
		* @note	根据冲量改变质点状态。
		*/
		void ApplyImpulse();
		dtkT3<double> GetAndClearImpulse();

		const GK::Point3& GetPoint(){ return mPts->GetPoint(mID); }
		dtkPoints::Ptr GetPoints() { return mPts; }
        dtkID GetPointID() { return mID; }
		void SetPointID(dtkID id) {mID = id;}

		//获取位置，用于更新
		dtkT3<double> GetPosition(ItrMethod method = Euler, dtkID iteration = 0);
        dtkT3<double> GetLastFramePosition() { return mPosLastFrame; }

		//更新位置
		/**
		* @brief		更新质点位置
		* @param[in]	newPos : 新位置
		* @param[in]	passToTwin : 更新邻居	
		* @note twin是当前点有同步关系的点
		*/				
		void SetPosition(dtkT3<double> newPos, bool passToTwin = true ) 
		{
			mPts->SetPoint(this->mID,GK::Point3(newPos[0],newPos[1],newPos[2]));
			if( passToTwin && mTwins.size() > 0)
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetPosition( newPos, false );
				}
			}
		}
        void SetPosition(dtkT3<double> newPos, dtkID iteration);

		dtkT3<double> GetVel(ItrMethod method = Euler, dtkID iteration = 0);
		dtkT3<double> GetAccel(ItrMethod method = Euler, dtkID iteration = 0);

		// this cannot be used during the update
		void SetVel(dtkT3<double> newVel, bool passToTwin = true ) { 
			mVel = newVel;
			/*if(  passToTwin && mTwins.size() > 0 )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetVel( newVel, false );
				}
			}*/
		}

        void SetVel(dtkT3<double> newVel, dtkID iteration);

		// this cannot be used during the update
		void SetPoint( GK::Point3& newPos, bool passToTwin = true ) {
			mPts->SetPoint( this->mID, newPos );
			/*if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetPoint( newPos, false );
				}
			}*/
		}

		//修改质量
		void SetMass(const double& mass){ mMass = mass; }
		const double& GetMass(){ return mMass; }

		//修改阻力系数
		void SetResistCoef(const double& resistCoef){ mResistCoef = resistCoef; }
		const double& GetResistCoef(){ return mResistCoef; }

		//恒力
		void SetForceDecorator(const dtkT3<double>& fd){ mForceDecorator = fd; }
		void AddForceDecorator(const dtkT3<double>& newFD) { mForceDecorator = mForceDecorator + newFD; }
		const dtkT3<double>& GetForceDecorator(){ return mForceDecorator; }

		//冲量
        void SetImpulse( const dtkT3<double>& impulse, bool passToTwin = true ) { 
            mImpulse = impulse;
			if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->SetImpulse( impulse, false );
				}
			}
        }
        void AddImpulse( const dtkT3<double>& newImpulse, bool passToTwin = true ) { 
            mImpulse = mImpulse + newImpulse; 
            mImpulseNum++;
            if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->AddImpulse( newImpulse, false );
				}
			}
        }
        const dtkT3<double>& GetImpulse() { return mImpulse; }

        void AddForce(const dtkT3<double>& f, bool passToTwin = true){ 
			mForceAccum = mForceAccum + f / (double)( mTwins.size() + 1 ); 
			if( mTwins.size() > 0 && passToTwin )
			{
				for( dtkID i = 0; i < mTwins.size(); i++ )
				{
					mTwins[i]->AddForce( f, false );
				}
			}
        }

        const dtkT3<double>& GetForceAccum() { return mForceAccum; }
        void SetForceAccum(dtkT3<double> newForceAccum) { mForceAccum = newForceAccum; }

		void SetActive( bool newActive, bool passToTwin = true );
        bool IsActive() { return mActive; }

		void SetCollide(bool newCollide) { mCollide = newCollide; }
		bool GetCollide() { return mCollide; }

		//点阻尼
		void SetPointDamp(double newPointDamp) { mPointDamp = newPointDamp; }
		double GetPointDamp() { return mPointDamp; }

		//重力
		void SetGravity( dtkT3<double> gravity ) { mGravity = gravity; }
        dtkT3<double> GetGravity() { return mGravity; }
		
		// the list of vector represent the mass point state in each iteration. 
        std::vector< dtkT3<double> > GetPosBuffer() { return mPosBuffers; }
        std::vector< dtkT3<double> > GetVelBuffer() { return mVelBuffers; }
        std::vector< dtkT3<double> > GetAccelBuffer() { return mAccelBuffers; }

//		std::vector< dtkT3<double> > mPosBuffers;
  //      dtkT3<double> mImpulse;

        void AddTwin( dtkPhysMassPoint* newTwin ) 
		{ 
			mTwins.push_back( newTwin ); 
		}

		void AbandonTwins()
		{
			mTwins.clear();
		}

  //      dtkPhysMassPoint* GetTwin() 
		//{ 
		//	return mTwin; 
		//}

        bool HasTwin() 
		{ 
            if( mTwins.size() != 0 )
                return true;
            else
                return false;
        }

		dtkID GetLabel() { return mLabel; }

		void SetLabel( dtkID label ) 
		{ 
			// avoid using 0 as label
			assert( label != 0 );
			mLabel = label; 
		}

		//停止
		void ResetDynamicState()
		{
			mVel = 0;
			mAccel = 0;
			for( dtkID i = 0; i < 3; i++ )
			{
				mVelBuffers[i] = 0;
				mAccelBuffers[i] = 0;
			}
		}

	private:
		
		dtkID mID;
		// the position at the last iteration.
		dtkPoints::Ptr mPts;

		dtkT3<double> mVel;  /**< 质点速度 */
		dtkT3<double> mAccel; /**< 质点加速度 */
		dtkT3<double> mForceAccum; /**< 合外力 */
		dtkT3<double> mGravity; /**< 重力 *//
        dtkT3<double> mResistance; /**< 阻力 */

		// the position at the last iteration.
        dtkT3<double> mPosLastFrame; /**< 上一帧位置 */
		double mMass;  /**< 质量 */
        double mResistCoef; /**< 阻力系数 */
		double mPointDamp; /**< 阻尼 */ 

		std::vector< dtkT3<double> > mVelBuffers;
		std::vector< dtkT3<double> > mAccelBuffers;
		std::vector< dtkT3<double> > mPosBuffers;

		dtkT3<double> mForceDecorator;	//已有力

        size_t mImpulseNum;  /**< 冲量数 */
        dtkT3<double> mImpulse;/**< 冲量 */
 
		bool mActive; /**< 质点是否活动，速度是否为零 */
		bool mCollide; /**< 是否发生碰撞 */

		dtkID mLabel;

	public:
		// Test Feature
		std::vector< dtkPhysMassPoint* > mTwins;
	};
}

#endif
