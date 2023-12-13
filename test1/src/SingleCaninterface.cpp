#include "SingleCaninterface.h"

SCanInterface* SCanInterface::instance = nullptr;

SCanInterface::SCanInterface()
{
    // 在构造函数中进行初始化，如果需要的话
    //controlCAN = std::make_shared<TControlCanFactor>(CanType::1,1);
}

SCanInterface* SCanInterface::getInstance()
{
    if (instance == nullptr)
    {
        instance = new SCanInterface();
    }
    return instance;
}

bool SCanInterface::open() const
{
    // 执行打开操作的实现
    // 返回操作结果
}

bool SCanInterface::close() const
{
    // 执行关闭操作的实现
    // 返回操作结果
}

bool SCanInterface::getParameter(uint32_t* parameterList, int parameterType, int nodeCount)
{
    // 执行获取参数操作的实现
    // 返回操作结果
}

bool SCanInterface::getParameter(uint8_t* nodeList, uint32_t* parameterList, int parameterType, int nodeCount)
{
    // 执行获取参数操作的实现
    // 返回操作结果
}

bool SCanInterface::setParameter(uint32_t* parameterList, int parameterType, int nodeCount)
{
    // 执行设置参数操作的实现
    // 返回操作结果
}

bool SCanInterface::setParameter(uint8_t* nodeList, uint32_t* parameterList, int parameterType, int nodeCount)
{
    // 执行设置参数操作的实现
    // 返回操作结果
}
