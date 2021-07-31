/* 
  SENSEI32 - Stand Alone Firmware V001 - Obtains data from the embedded sensors, manage deep-sleep and pubblish hourly to ThingSpeak

  ####################################################################################################################################
  This software, the ideas and concepts is Copyright (c) Davide Raggini & Davide Samori 2021. All rights to this software are reserved.

  Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author Davide Raggini & Davide Samori as the source of the material.
  3. You may not, except with our express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

  The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
  software use is visible to an end-user.

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  Contact us:   davide.raggini@gmail.com
                davide.samori@gmail.com
  
  ####################################################################################################################################
  This software uses ideas and concepts which are Copyright (c) David Bird 2018. All rights to this software are reserved.

  Any redistribution or reproduction of any part or all of the contents in any form is prohibited other than the following:
  1. You may print or download to a local hard disk extracts for your personal and non-commercial use only.
  2. You may copy the content to individual third parties for their personal use, but only if you acknowledge the author David Bird as the source of the material.
  3. You may not, except with my express written permission, distribute or commercially exploit the content.
  4. You may not transmit it or store it in any other website or other form of electronic retrieval system for commercial purposes.

  The above copyright ('as annotated') notice and this permission notice shall be included in all copies or substantial portions of the Software and where the
  software use is visible to an end-user.

  THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT. FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY
  OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHOR OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  See more at http://www.dsbird.org.uk
*/

#ifndef THINGSPEAK_PUSH_H_
#define THINGSPEAK_PUSH_H_


/*******************************************************************************************
 *
 * Dependencies
 *
 *******************************************************************************************/

#include <Arduino.h>
#include <HTTPClient.h>

/*******************************************************************************************
 *
 * Thingspeak Functions
 *
 *******************************************************************************************/

/**
  * @brief  Push to Thingspeak worker function
  *
  * @param  None
  * @retval None
  */
bool ThingSpeakSend(WiFiClient& client, float t, float ru, float pres, float gas, float vbat) {
  
  client.stop(); // close connection before sending a new request
  HTTPClient http;
  // Your Domain name with URL path or IP address with path
  http.begin(ts_server);
  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  // Data to send with HTTP POST
  String httpPostData = "api_key=" + ts_apikey + "&field1=" + String(t) + "&field2=" + String(ru) + "&field3=" + String(pres) + "&field4=" + String(gas) + "&field5=" + String(vbat);           
  // Send HTTP POST request
  int httpCode = http.POST(httpPostData);
  if(httpCode == HTTP_CODE_OK) {
    Serial.println("[" + String(millis()) + "] ThingSpeak: POST OK");
    client.stop();
    http.end();
    return true;
  }
  else
  {
    Serial.println("[" + String(millis()) + "] ThingSpeak: POST FAIL");
    client.stop();
    http.end();
    return false;
  }
  http.end();
  return true;
  
}


#endif /* ifndef THINGSPEAK_PUSH_H_ */
