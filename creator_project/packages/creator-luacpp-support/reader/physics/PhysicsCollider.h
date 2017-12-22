/****************************************************************************
 Copyright (c) 2017 Chukong Technologies Inc.
 
 http://www.cocos2d-x.org
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ****************************************************************************/
#pragma once

#include <vector>

#include "2d/CCNode.h"
#include "math/Vec2.h"
#include "math/CCGeometry.h"
#include "../Macros.h"

class b2Shape;
class b2Fixture;

NS_CCR_BEGIN

class RigidBody;
/****************************************************************************
 PhysicsCollider
 ****************************************************************************/
class PhysicsCollider
{
public:
    float getDensity() const;
    void setDensity(float density);
    
    float getFriction() const;
    void setFriction(float friction);
    
    bool isSensor() const;
    void setSensor(bool flag);
    
    float getRestitution() const;
    void setRestitution(float restitution);
    
    void apply();
    cocos2d::Rect getAABB() const;
    
    RigidBody* getBody() const;
    
protected:
    struct ColliderInfoCommon
    {
        bool sensor;
        float density;
        float friction;
        float restitution;
        cocos2d::Vec2 offset;
    };
    
    PhysicsCollider(RigidBody *_rigidBody, const std::vector<bool>& collisionMatrix, const ColliderInfoCommon& colliderInfo);
    ~PhysicsCollider();
    
    virtual std::vector<b2Shape*> createShapes(float scale) = 0;
    float getNodeWorldScale(const cocos2d::Node* node) const;
    
    cocos2d::Vec2 _offset;
    
private:
    void destroy();
    void init();
    
    float _density;
    float _friction;
    float _restitution;
    bool _sensor;
    std::vector<bool> _collisionMatrix;
    RigidBody *_rigidBody;
    std::vector<b2Fixture*> _fixtures;
    bool _inited;
    cocos2d::Rect _rect;
};

/****************************************************************************
 PhysicsColliderBox
 ****************************************************************************/
class PhysicsColliderBox : public PhysicsCollider
{
private:
    struct ColliderInfo : public ColliderInfoCommon
    {
        cocos2d::Size size;
    };
    
    PhysicsColliderBox(RigidBody *rigidBody, const std::vector<bool>& collisionMatrix, const struct ColliderInfo& colliderInfo);
    virtual std::vector<b2Shape*> createShapes(float scale) override;

    cocos2d::Size _size;
};

/****************************************************************************
 PhysicsColliderCircle
 ****************************************************************************/
class PhysicsColliderCircle : public PhysicsCollider
{
private:
    struct ColliderInfo : public ColliderInfoCommon
    {
        float radius;
    };
    
    PhysicsColliderCircle(RigidBody *rigidBody, const std::vector<bool>& collisionMatrix, const struct ColliderInfo& colliderInfo);
    virtual std::vector<b2Shape*> createShapes(float scale) override;
    
    float _radius;
    
};

/****************************************************************************
 PhysicsColliderChain
 ****************************************************************************/
class PhysicsColliderChain : public PhysicsCollider
{
private:
    struct ColliderInfo : public ColliderInfoCommon
    {
        bool loop;
        std::vector<cocos2d::Vec2> points;
    };
    
    PhysicsColliderChain(RigidBody *rigidBody, const std::vector<bool>& collisionMatrix, const struct ColliderInfo& colliderInfo);
    virtual std::vector<b2Shape*> createShapes(float scale) override;
    
    bool _loop;
    std::vector<cocos2d::Vec2> _points;
};

/****************************************************************************
 PhysicsColliderPolygon
 ****************************************************************************/
// TODO

NS_CCR_END
