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
#include "PhysicsManager.h"
#include <Box2D/Dynamics/b2World.h>
#include "base/CCScheduler.h"
#include "base/CCDirector.h"

NS_CCR_BEGIN

#define VELOCITY_ITERATIONS     10
#define POSITION_ITERATIONS     10
#define FIXED_TIME_STEP         1/60.f
#define MAX_ACCUMULATOR         1/5.f

PhysicsManager* PhysicsManager::s_instance = nullptr;

void PhysicsManager::update(float dt)
{
    //TODO
    // emit "before-step"
    
    _stepping = true;
    {
        if (_accumulatorEnabled)
        {
            _accumulator += dt;
            
            if (_accumulator > MAX_ACCUMULATOR)
                _accumulator = MAX_ACCUMULATOR;
            
            while (_accumulator > FIXED_TIME_STEP)
            {
                _world->Step(FIXED_TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
                _accumulator -= FIXED_TIME_STEP;
            }
        }
        else
        {
            auto timeStep = cocos2d::Director::getInstance()->getFrameRate();
            _world->Step(timeStep, VELOCITY_ITERATIONS, POSITION_ITERATIONS);
        }
    }
    _stepping = false;
    
    //TODO
    // _world->DrawDebugData();
    
    syncNode();
}


//
// private functions
//

PhysicsManager* PhysicsManager::getInstance()
{
    if (!s_instance)
        s_instance = new PhysicsManager();
    
    return s_instance;
}

void PhysicsManager::destroy()
{
    delete s_instance;
    s_instance = nullptr;
}

PhysicsManager::PhysicsManager()
: _enabled(true)
, _stepping(false)
, _accumulatorEnabled(false)
, _accumulator(0.f)
{
    _world = new b2World({0, -10});
    
    startUpdate();
}

PhysicsManager::~PhysicsManager()
{
    endUpdate();
    
    //TODO: delete all rigid bodies, colliders, contracts?
    
    delete _world;
    _world = nullptr;
}

void PhysicsManager::startUpdate()
{
    cocos2d::Director::getInstance()->getScheduler()->scheduleUpdate(this, 1, false);
}

void PhysicsManager::endUpdate()
{
    cocos2d::Director::getInstance()->getScheduler()->unscheduleUpdate(this);
}

void PhysicsManager::syncNode()
{
    
}

NS_CCR_END
