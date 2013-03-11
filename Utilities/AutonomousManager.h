//
//  autonomous.h
//  First 2013
//
//  Created by Kyle Rokos on 1/20/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__AutonomousManager__
#define __First_2013__AutonomousManager__

#include <iostream>
#include <vector>
#include "CxTimer.h"

class NextState
{
public:
    NextState() : m_nextState(0),m_timeUntilNextState(0), m_timeout(0) {};
    
    NextState(int32_t nextState, uint32_t timeUntilNextState, uint32_t timeout=60)
    : m_nextState(nextState), m_timeUntilNextState(timeUntilNextState*100), m_timeout(timeout*100) {};
    
    int32_t NextStateID() {return m_nextState;}
    uint32_t TimeUntilNextState() {return m_timeUntilNextState;}
    uint32_t Timeout() {return m_timeout;}
    
    static NextState EndState() { return NextState(-1,0);};
    
private:
    int32_t m_nextState;
    uint32_t m_timeUntilNextState;
    uint32_t m_timeout;
};


template<class Robot>
class AutonomousManager
{
public:
    typedef NextState (*stateFunction)(Robot*,int32_t);
    
public:
    AutonomousManager(Robot *closure, stateFunction stateFunction, int32_t initialState=0);
    ~AutonomousManager();
    
    void SetStartState(stateFunction function, int32_t initialState);
    
    void Run();
    
private:
    Robot *m_closure;
    stateFunction m_stateFunction;
    int32_t m_nextState;
    uint32_t m_timeUntilNextState;
    uint32_t m_timeout;
    CxTimer m_minTimer;
    CxTimer m_maxTimer;
};

template<class Robot>
AutonomousManager<Robot>::AutonomousManager(Robot *closure, stateFunction stateFunction, int32_t initialState)
    : m_closure(closure), m_nextState(initialState), m_timeUntilNextState(0), m_timeout(0)
{
    m_minTimer.Reset();
    m_maxTimer.Reset();	
}
    
template<class Robot>
AutonomousManager<Robot>::~AutonomousManager()
{

}

template<class Robot>
void AutonomousManager<Robot>::SetStartState(stateFunction stateFunction, int32_t initialState)
{
    m_stateFunction = stateFunction;
    m_nextState = initialState;
    m_minTimer.Reset();
    m_maxTimer.Reset();
}

template<class Robot>
void AutonomousManager<Robot>::Run()
{
    NextState info;
    bool finished = m_closure->UpdateSubsystems();
    
    if(m_nextState==-1)
        return;
    
    // continue on to next state when both conditions are true:
    // 1. all commands have finished
    // 2. minimum time has elapsed
    // or when the following condition is true:
    // 1. maximum time has elapsed (timeout)
    if((finished && m_minTimer.CkTime(true, m_timeUntilNextState)) || m_maxTimer.CkTime(true, m_timeout))
    {
        info = m_stateFunction(m_closure, m_nextState);
        m_minTimer.Reset();
        m_maxTimer.Reset();
        m_nextState = info.NextStateID();
        m_timeUntilNextState = info.TimeUntilNextState();
        m_timeout = info.Timeout();
    }
}


#endif /* defined(__First_2013__AutonomousManager__) */
