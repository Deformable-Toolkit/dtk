/*
 * @Author: tom: https://github.com/TOMsworkspace 
 * @Date: 2021-08-05 16:52:58 
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-08-05 21:52:46
 */

#ifndef DTK_RIGIBODY_H
#define DTK_RIGIBODY_H

#include <vector>
#include <memory>

#include "dtkTx.h"
#include "dtkdtkGraphicsKernel.h"
#include "dtkIDTypes.h"
#include "dtkMatrix.h"

namespace dtk {

    // 刚体
    class dtkRigidBody {
    public:
        using vertex_list = std::vector<dtkDouble2>;
        using ptr = std::shared_ptr<dtkRigidBody>;

        dtkRigidBody(dtkID id, double mass);
        virtual ~dtkRigidBody() = default;

        bool can_collide(const dtkRigidBody &other) const;
        void update_impulse(const dtkDouble2 &impulse, const dtkDouble2 &r);
        void update_force(const dtkDouble2 &gravity, double dt);

        dtkDouble2 local_to_world(const dtkDouble2 &) const;

        dtkID get_id() const;

        double get_mass() const;
        void set_mass(double);

        double get_inv_mass() const;

        double get_inertia() const;
        void set_inertia(double);

        double get_inv_inertia() const;

        const dtkDouble2 &getmCentroid() const;
        void set_centroid(const dtkDouble2 &);

        const dtkDouble2 &get_position() const;
        void set_position(const dtkDouble2 &);

        const dtkMatrix22 &get_rotation() const;
        void set_rotation(const dtkMatrix22 &);

        const dtkDouble2 &get_velocity() const;
        void set_velocity(const dtkDouble2 &);

        double get_angular_velocity() const;
        void set_angular_velocity(double);

        const dtkDouble2 &get_force() const;
        void set_force(const dtkDouble2 &);

        double get_torque() const;
        void set_torque(double);

        double get_friction() const;
        void set_friction(double);

    protected:
        //DISALLOW_COPY_AND_ASSIGN(dtkRigidBody)

        dtkRigidBody(const dtkRigidBody &) = delete;
        const dtkRigidBody & operator=(const dtkRigidBody &) = delete;

        dtkID mId; // id
        double mMass; // 质量
        double invMass; // 质量倒数（缓存，用于运算）
        double mInertia; // 转动惯量
        double invInertia; // 转动惯性倒数（缓存，用于运算）
        dtkDouble2 mCentroid; // 重心
        dtkDouble2 mPosition; // 位置
        dtkMatrix22 mRotation; // 旋转矩阵
        dtkDouble2 mVelocity; // 速度
        double mAngularVelocity; // 角速度
        dtkDouble2 mForce; // 受力
        double mTorque; // 扭矩
        double mFriction; // 摩擦力
    };

    // 几何刚体
    class dtkPolygonRigidBody : public dtkRigidBody {
    public:
        using ptr = std::shared_ptr<dtkPolygonRigidBody>;

        dtkPolygonRigidBody(dtkID id, double mass, const vertex_list& vertices);

        size_t count() const;

        dtkDouble2 operator[](size_t idx) const;
        dtkDouble2 edge(size_t idx) const;

        double min_separating_axis(size_t& idx, const dtkPolygonRigidBody& other) const;

    protected:
        //DISALLOW_COPY_AND_ASSIGN(dtkPolygonRigidBody)

        dtkPolygonRigidBody(const dtkPolygonRigidBody &) = delete;
        const dtkPolygonRigidBody & operator=(const dtkPolygonRigidBody &) = delete;

        vertex_list mVertices;
    };
}

#endif //DTK_RIGIBODY_H