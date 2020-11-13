#ifdef __PLAT_NATIVE_SIM__

#include "user_interface.h"
#include "execution_control.h"

bool Kernel::initialised = false;

bool Kernel::execute_loop( uint64_t max_end_ticks) {
  //simulation time lock
  updateRealtime();
  if (getRealtimeTicks() > getTicks()) {
    realtime_nanos = nanos();
  } else while (getTicks() > getRealtimeTicks()) {
    updateRealtime();
    std::this_thread::yield();
  }

  //todo: investigate dataloss when pulling from SerialMonitor rather than pushing from here

  // Marlin often gets into reentrant loops, this is the only way to unroll out of that call stack early
  if (quit_requested) throw (std::runtime_error("Quit Requested"));

  if (usb_serial.transmit_buffer.available()) {
    char buffer[usb_serial.transmit_buffer_size];
    auto count = usb_serial.transmit_buffer.read((uint8_t *)buffer, usb_serial.transmit_buffer_size - 1);
    buffer[count] = '\0';
    std::dynamic_pointer_cast<SerialMonitor>(UserInterface::ui_elements["Serial Monitor"])->insert_text(buffer);
  }

  uint64_t current_ticks = getTicks();
  uint64_t current_priority = 99;
  if (isr_stack.size()) {
    current_priority = isr_stack.back()->priority;
  }

  uint64_t lowest_isr = std::numeric_limits<uint64_t>::max();
  KernelTimer* next_isr = nullptr;
  for (auto& timer : timers) {
    uint64_t value = timer.next_interrupt(frequency);
    if (timers_active && value < lowest_isr && value < max_end_ticks && timer.enabled() && !timer.running && timer.priority < current_priority) {
      lowest_isr = value;
      next_isr = &timer;
    }
  }

  if (next_isr != nullptr ) {
    if (current_ticks > lowest_isr) {
      isr_timing_error = current_ticks - lowest_isr;
      next_isr->source_offset = current_ticks; // late interrupt
    } else {
      next_isr->source_offset = next_isr->next_interrupt(frequency); // timer was reset when the interrupt fired
      isr_timing_error = 0;
    }
    setTicks(next_isr->source_offset);
    isr_stack.push_back(next_isr);
    next_isr->execute();
    isr_stack.pop_back();
    return true;
  }

  return false;
}

// if a thread wants to wait, see what should be executed during that wait
void Kernel::delayCycles(uint64_t cycles) {
  auto end = getTicks() + cycles;
  while (execute_loop(end) && getTicks() < end);
  if (end > getTicks()) setTicks(end);
}

// this is needed for when marlin loops idle waiting for an event with no delays (syncronize)
void Kernel::yield() {
  if(isr_stack.size() == 0) {
    // Kernel not started?
    setTicks(getTicks() + nanosToTicks(100));
    return;
  }
  auto max_yield = isr_stack.back()->next_interrupt(frequency);
  if(!execute_loop(max_yield)) { // dont wait longer than this threads exec period
    setTicks(max_yield);
    isr_stack.back()->source_offset = max_yield; // there was nothing to run, and we now overrun our next cycle.
  }
}


#endif // __PLAT_NATIVE_SIM__
