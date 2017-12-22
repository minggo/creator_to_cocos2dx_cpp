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
#include "math/Vec2.h"
#include "../Macros.h"
#include "RigidBody.h"

class b2World;
class b2Fixture;

NS_CCR_BEGIN

class PhysicsCollider;
class RigidBody;

struct RaycastResult
{
    PhysicsCollider* collider;
    cocos2d::Vec2 point;
    cocos2d::Vec2 normal;
    float fraction;
};

enum class RayCastType
{
    CLOSEST,
    ANY,
    ALL_CLOSEST,
    ALL
};

class PhysicsManager
{
public:
    PhysicsCollider* testPoint(const cocos2d::Vec2& point) const;
    std::vector<PhysicsCollider*> testAABB(const cocos2d::Rect& rect) const;
    std::vector<RaycastResult> rayCast(const cocos2d::Vec2& point1, const cocos2d::Vec2& point2, RayCastType type) const;
    
    bool isEnabled() const;
    void setEnabled(bool flag);
    
    // Gravity is measured in points/s.
    cocos2d::Vec2 getGravity() const;
    void setGravity(const cocos2d::Vec2& gravity);
    
    void update(float dt);
private:
    friend class PhysicsCollider;
    
    static PhysicsManager* getInstance();
    static void destroy();
    
    PhysicsManager();
    ~PhysicsManager();
    CREATOR_DISALLOW_COPY_ASSIGN_AND_MOVE(PhysicsManager);
    void registerContactFixture(b2Fixture*);
    void unregisterContactFixture(b2Fixture*);
    
    void addBody(cocos2d::Node* node, unsigned int nodeGroupIndex, RigidBody::Type type, bool enableContractListener
                 bool bullet, bool sleepingAllowed, float gravityScale, );
    
    void startUpdate();
    void endUpdate();
    void syncNode();
    
    static PhysicsManager *s_instance;
    b2World* _world;
    std::vector<RigidBody*> _bodies;
    // Fixtures that need to be reported.
    std::vector<b2Fixture*> _contactFixtures;
    bool _enabled;
    bool _stepping;
    bool _accumulatorEnabled;
    float _accumulator;
};

NS_CCR_END
