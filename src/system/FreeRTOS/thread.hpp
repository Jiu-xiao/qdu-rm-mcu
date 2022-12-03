#pragma once

#include <stdint.h>

#include <string>

#include "FreeRTOS.h"
#include "task.h"

#define THREAD_DECLEAR(_handle, _fn, _stack_depth, _priority, _arg) \
  do {                                                              \
    _handle.Create(_fn, #_fn, _stack_depth, _priority, _arg);       \
  } while (0)

namespace System {
class Thread {
 private:
  template <typename Arg>
  class HelperFunction {
   public:
    HelperFunction(void (*fun)(Arg arg), Arg arg) : fun_(fun), arg_(arg) {}

    static void Call(void* arg) {
      auto self = static_cast<HelperFunction*>(arg);
      self->fun_(self->arg_);
    }

    void (*fun_)(Arg arg);
    Arg arg_;
  };

 public:
  typedef enum { IDLE, Low, Medium, High, Realtime } Priority;

  template <typename Fun, typename Arg>
  /**
   * 创建线程
   * @see xTaskCreate
   * @tparam Fun 线程入口函数的类型
   * @tparam Arg 向线程入口函数传递的参数类型
   * @param fn 指向线程入口函数的指针
   * @param arg 线程入口函数的参数
   * @param name 线程的名称
   * @param stack_depth 栈深度
   * @param priority 线程优先级
   */
  void Create(Fun fn, Arg arg, const char* name, uint32_t stack_depth,
              Priority priority) {
    static_cast<void (*)(Arg)>(fn);
    /* overload new operator by manual */
    HelperFunction<Arg>* helper_fn = static_cast<HelperFunction<Arg>*>(
        pvPortMalloc(sizeof(HelperFunction<Arg>)));

    *helper_fn = HelperFunction<Arg>(fn, arg);

    xTaskCreate(HelperFunction<Arg>::Call, name, stack_depth, helper_fn,
                priority, &(this->handle_));
  }

  static void Sleep(uint32_t microseconds) { vTaskDelay(microseconds); }

  void Stop();

  static uint32_t GetTick() { return xTaskGetTickCount(); }

  static void StartKernel();

 private:
  TaskHandle_t handle_ = NULL;
};
}  // namespace System
