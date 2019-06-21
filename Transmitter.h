void SetupTransmitter(int pin)
{
	vw_set_tx_pin(pin);
	/* Set the transmit logic level (LOW = transmit for this 
     version of module)*/ 
	vw_set_ptt_inverted(true);   
	/* Transmit at 2000 bits per second */
	vw_setup(2000);    // Bits per sec
}

void SendMsg(unsigned int Data)
{
  /* The transmit buffer that will hold the data to be 
     transmitted. */
  byte TxBuffer[2];
  

  /* ...and store it as high and low bytes in the transmit 
     buffer */
  TxBuffer[0] = Data >> 8;
  TxBuffer[1] = Data;
  
  /* Turn on the LED on pin 13 to indicate that we are about 
    to transmit data */
  // digitalWrite(13, HIGH); 
  /* Send the data (2 bytes) */
  vw_send((byte *)TxBuffer, 2);
  /* Wait until the data has been sent */
  vw_wait_tx(); 
  
  /* Turn off the LED on pin 13 to indicate that we have 
     now sent the data */
  // digitalWrite(13, LOW); 
  
  /* Do nothing for a second. Lower this delay to send 
     data quicker */
  delay(1000);
}