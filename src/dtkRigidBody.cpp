/*
 * @Author: tom: https://github.com/TOMsworkspace 
 * @Date: 2021-08-05 17:12:59 
 * @Last Modified by: tom: https://github.com/TOMsworkspace
 * @Last Modified time: 2021-08-05 21:15:32
 */
#include "dtkRigidBody.h"

namespace dtk
{

    dtkRigidBody::dtkRigidBody(dtkID id, double mass) : mId(id), mRotation(glm::mat2x2(1.0)), mAngularVelocity(0.0), mTorque(0.0), mFriction(1.0)
    {
        set_mass(mass);
    }

    // 判断碰撞条件
    bool dtkRigidBody::can_collide(const dtkRigidBody &other) const
    {
        // 只要质量不是无穷大，都可以发生碰撞
        // 当且仅当两者都是无穷大时，不检测碰撞
        // 这种情况是当：两者为边界时
        return !(std::isinf(mMass) && std::isinf(other.mMass));
    }

    // 更新动量
    void dtkRigidBody::update_impulse(const dtkDouble2 &impulse, const dtkDouble2 &r)
    {
        if (std::isinf(mMass))
            return;
        // 线动量：P = m * v, v += delta_P / m
        mVelocity += impulse * invMass;
        // 角动量：omega = I * beta, beta += delta_I / omega
        // omega = r X P
        mAngularVelocity += invInertia * cross(r, impulse);
    }

    // 更新力
    void dtkRigidBody::update_force(const dtkDouble2 &gravity, double dt)
    {
        if (std::isinf(mMass))
            return;
        // 合外力 = 质量 × 线加速度 = 线动量的变化率
        // V += a * t, a = g + F / m
        mVelocity += (gravity + mForce * invMass) * dt;
        // 合外力矩 = 转动惯量 × 角加速度 = 角动量的变化率
        // beta += (M / I) * t
        mAngularVelocity += (mTorque * invInertia) * dt;
        // s += v * t
        mPosition += mVelocity * dt;
        // theta += beta * t

        double theta = mAngularVelocity * dt;
        auto _sin = std::sin(theta);
        auto _cos = std::cos(theta);
        glm::mat2x2 rotatemt(_cos, -_sin, _sin, -_cos);
        //return rotatemt * mRotation;

        mRotation = rotate(mAngularVelocity * dt) * mRotation;
    }

    dtkDouble2 dtkRigidBody::local_to_world(const dtkDouble2 &local_point) const
    {
        return mPosition + local_point;
    }

    dtkID dtkRigidBody::get_id() const
    {
        return mId;
    }

    double dtkRigidBody::get_mass() const
    {
        return mMass;
    }

    void dtkRigidBody::set_mass(double mass)
    {
        mMass = mass;
        invMass = 1 / mMass;
    }

    double dtkRigidBody::get_inv_mass() const
    {
        return invMass;
    }

    double dtkRigidBody::get_inertia() const
    {
        return mInertia;
    }

    void dtkRigidBody::set_inertia(double inertia)
    {
        mInertia = inertia;
        invInertia = std::isinf(mInertia) ? 0 : 1 / mInertia;
    }

    double dtkRigidBody::get_inv_inertia() const
    {
        return invInertia;
    }

    const dtkDouble2 &dtkRigidBody::get_centroid() const
    {
        return mCentroid;
    }

    void dtkRigidBody::set_centroid(const dtkDouble2 &centroid)
    {
        mCentroid = centroid;
    }

    const dtkDouble2 &dtkRigidBody::get_position() const
    {
        return mPosition;
    }

    void dtkRigidBody::set_position(const dtkDouble2 &position)
    {
        mPosition = position;
    }

    const dtkMatrix22 &dtkRigidBody::get_rotation() const
    {
        return mRotation;
    }

    void dtkRigidBody::set_rotation(const dtkMatrix22 &rotation)
    {
        mRotation = rotation;
    }

    const dtkDouble2 &dtkRigidBody::get_velocity() const
    {
        return mVelocity;
    }

    void dtkRigidBody::set_velocity(const dtkDouble2 &velocity)
    {
        mVelocity = velocity;
    }

    double dtkRigidBody::get_angular_velocity() const
    {
        return mAngularVelocity;
    }

    void dtkRigidBody::set_angular_velocity(double angular_velocity)
    {
        mAngularVelocity = angular_velocity;
    }

    const dtkDouble2 &dtkRigidBody::get_force() const
    {
        return mForce;
    }

    void dtkRigidBody::set_force(const dtkDouble2 &force)
    {
        mForce = force;
    }

    double dtkRigidBody::get_torque() const
    {
        return mTorque;
    }

    void dtkRigidBody::set_torque(double torque)
    {
        mTorque = torque;
    }

    double dtkRigidBody::get_friction() const
    {
        return mFriction;
    }

    void dtkRigidBody::set_friction(double friction)
    {
        mFriction = friction;
    }

    // ---------------------------------------------------
    // dtkPolygonRigidBody

    // 计算多边形面积 
    static double calc_polygon_area(const std::vector<dtkDouble2> &vertices)
    {
        double area = 0;
        auto size = vertices.size();
        // 求所有三角形的面积之和 
        for (size_t i = 0; i < size; ++i)
        {
            auto j = (i + 1) % size;
            // 叉乘求两相邻向量所成平行四边形面积 
            // 所以要求三角形面积就要除以2
            area += cross(vertices[i], vertices[j]);
        }
        return area / 2;
    }

    // 计算多边形重心 
    static dtkDouble2 calc_polygon_centroid(const std::vector<dtkDouble2> &vertices)
    {
        dtkDouble2 gc;
        auto size = vertices.size();
        // 重心 = (各三角形重心 * 其面积) / 总面积 
        // 三角形重心 = 两向量之和 / 3 
        for (size_t i = 0; i < size; ++i)
        {
            auto j = (i + 1) % size;
            gc += (vertices[i] + vertices[j]) * cross(vertices[i], vertices[j]);
        }
        return gc / 6.0 / calc_polygon_area(vertices);
    }

    // 计算多边形转动惯量 
    static double calc_polygon_inertia(double mass, const dtkRigidBody::vertex_list &vertices)
    {
        double acc0 = 0, acc1 = 0;
        auto size = vertices.size();
        // 转动惯量 = m / 6 * (各三角形面积 * 其(a*a+a*b+b*b)) / (总面积) 
        for (size_t i = 0; i < size; ++i)
        {
            auto a = vertices[i], b = vertices[(i + 1) % size];
            auto _cross = std::abs(cross(a, b));
            acc0 += _cross * (dot(a, a) + dot(b, b) + dot(a, b));
            acc1 += _cross;
        }
        return mass * acc0 / 6 / acc1;
    }

    dtkPolygonRigidBody::dtkPolygonRigidBody(dtkID id, double mass, const dtkRigidBody::vertex_list &vertices)
        : dtkRigidBody(id, mass), mVertices(vertices)
    {
        set_inertia(calc_polygon_inertia(mass, vertices));
        set_centroid(calc_polygon_centroid(vertices));
    }

    size_t dtkPolygonRigidBody::count() const { return mVertices.size(); }

    dtkDouble2 dtkPolygonRigidBody::operator[](size_t idx) const
    {
        // 顶点进行旋转变换，返回旋转后位置.
         
        return mRotation * (mVertices[idx] - mCentroid) + mCentroid;
    }

    dtkDouble2 dtkPolygonRigidBody::edge(size_t idx) const
    {
        return (*this)[(idx + 1) % mVertices.size()] - (*this)[idx];
    }

    // 碰撞检测-SAT分离轴定理
    // 检测两凸包是否相交
    // 表述：如果两个凸多边形没有相交，那么存在这两个物体在一个轴上的投影不重叠. 
    // 轴：只需采用两个凸包的每个条做检测即可 
    double dtkPolygonRigidBody::min_separating_axis(size_t &idx, const dtkPolygonRigidBody &other) const
    {
        double separation = -inf;
        // 遍历几何物体A的所有顶点 
        for (size_t i = 0; i < mVertices.size(); ++i)
        {
            // 获得A各顶点的世界坐标 
            auto va = local_to_world((*this)[i]);
            // 获得当前顶点到下一顶点的边的单位法向量 
            auto N = normal(edge(i));
            // 最小分离向量 
            auto min_sep = inf;
            // 遍历几何物体B
            for (size_t j = 0; j < other.count(); ++j)
            {
                // 获得B各顶点的世界坐标
                auto vb = other.local_to_world(other[j]);
                // vb - va = 从顶点A到顶点B的向量
                // normal  = 从顶点A到顶点A'的单位向量
                // dot(vb - va, normal) = 若点B到边AA'投影为P，结果为AP的长度
                // 由于这里取最小值，因此
                // min_sep = 以AA'边的法向量N为轴，将B物体各顶点所做投影的最小长度
                min_sep = std::min(min_sep, dot(vb - va, N));
            }
            if (min_sep > separation)
            {
                separation = min_sep; // 寻找最大间隙
                idx = i;              // 轴
            }
        }
        return separation; // 非负则表示不相交
    }
}
