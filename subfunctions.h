
int debouncePixyBlocks(int readBlocks, int bdelay) {
  // A function that reads a switch value from a specified pin
  // number and returns the most recent stable value from that
  // pin as either HIGH or LOW, based on a given bounce delay
  // (in msec). A switch index must be assigned to keep bounce
  // timing for each switch from overwriting one another.
  
  // Define variables to hold the most recent
  // switch value and last switch transition time
  static int prior_reading = LOW;
  static long prior_time = 0;

  // Define variable to hold current stable switch state
  // and initialize to be in the LOW state
  static int switch_state = LOW;

  // Read current switch value into local variable
  int current_reading = (readBlocks > 0);

  // Check for a change in switch state
  if (current_reading != prior_reading) {
    // If a change is detected in the switch value,
    // reset the delay clock and update the prior value
    prior_time = millis();
    prior_reading = current_reading;
  }

  // Has the switch potentially taken on a new value?
  if (switch_state != current_reading) {
    // Did the switch state remained unchanged long enough to
    // be considered debounced?
    if ((millis() - prior_time) >= bdelay) {
      // If so, accept updated switch value   
      switch_state = current_reading;
    } else {

    }
  }

  return switch_state;
}