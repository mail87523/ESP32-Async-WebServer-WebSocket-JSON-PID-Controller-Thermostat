/**
 * ----------------------------------------------------------------------------
 * ESP32 Web Controlled Thermostat
 * ----------------------------------------------------------------------------
 * Author: Stéphane Calderoni
 * Date:   April 2020
 * ----------------------------------------------------------------------------
 * This project is a response to a request made on the RNT Lab forum:
 * https://rntlab.com/question/java-script-code-to-refresh-home-page-only-once/
 * ----------------------------------------------------------------------------
 */

// ----------------------------------------------------------------------------
// Global constants
// ----------------------------------------------------------------------------

// Periodic temperature reading delay
const temperatureCaptureTime = 10000; // 10 seconds (in milliseconds)

// ----------------------------------------------------------------------------
// Global variables
// ----------------------------------------------------------------------------

// Default asynchronous request manager (classical AJAX method)
var xhttp = new XMLHttpRequest();

/**
 * We will need to read some data or update some elements of the HTML page.
 * So we need to define variables to reference them more easily throughout
 * the program.
 */

// Current temperature display screen
// ----------------------------------

var screen; // Container
var temperature; // HTML element that incorporates the current temperature value
var unit; // HTML element that incorporates the display of the temperature reading unit (°C)

// Input fields for temperature limits
// -----------------------------------
var Setpoint;
var Kp;
var Ki;
var Kd;
var Offset;
// Current time display
// --------------------

var time;

// ESP32 control buttons
// ---------------------

var btnDefault;
//var btnReboot;
var btnUpload;
var btnSave;
// ----------------------------------------------------------------------------
// Initialization on full loading of the HTML page
// ----------------------------------------------------------------------------

window.addEventListener("load", onLoad);

function onLoad(event) {
 
  initTemperatureDisplay();
  initTime();
  initThresholds();
  initButtons();
  initProbe();
  showPanel();
  initWebSocket();  
  initGPIOButton();
}

// ----------------------------------------------------------------------------
// Progressive appearance of the control panel when everything is ready
// ----------------------------------------------------------------------------

function showPanel() {
  let panel = document.getElementById("panel");
  panel.classList.add("showing");
  panel.addEventListener("animationend", () => {
    panel.style.opacity = 1;
  });
}

// ----------------------------------------------------------------------------
// Initialization of the current temperature display
// ----------------------------------------------------------------------------

function initTemperatureDisplay() {
  screen = document.getElementById("screen");
  temperature = document.getElementById("temperature");
  unit = document.getElementById("unit");
}

// ----------------------------------------------------------------------------
// Initialization of the current time display
// ----------------------------------------------------------------------------

function initTime() {
  time = document.getElementById("time");
  updateTime();
  // a timing event manager is initialized
  // which must be triggered every second to refresh
  // the display of the current time
  setInterval(updateTime, 1000);
}

function updateTime() {
  let now = new Date();
  let h = now.getHours();
  let m = now.getMinutes();
  let s = now.getSeconds();
  time.value = `${normalize(h)}:${normalize(m)}:${normalize(s)}`;
}

function normalize(digit) {
  return (digit < 10 ? "0" : "") + digit;
}

// ----------------------------------------------------------------------------
// Initialization and control of the input fields for temperature thresholds
// ----------------------------------------------------------------------------

function initThresholds() {
  Setpoint = document.getElementById("Setpoint");
  Kp = document.getElementById("Kp");
  Ki = document.getElementById("Ki");
  Kd = document.getElementById("Kd");
  Offset = document.getElementById("Offset");
}

// Event managers related to the entry of new temperature thresholds
// -----------------------------------------------------------------

// Once the entry is complete
// --------------------------

/**
 * Here we must make sure that the temperature thresholds have been entered correctly:
 * the user may have inverted the min and max when entering them, in which case they
 * must be put back in the right order.
 *
 * Once this check has been made, the display screen of the current temperature is
 * refreshed to take into account the new thresholds, and change the display colour
 * if necessary.
 *
 * Finally, the new thresholds are transmitted to the ESP32 to be saved in the EEPROM.
 */

function saveThresholds() {
  let sp = Number.parseFloat(Setpoint.value);
  let kp = Number.parseFloat(Kp.value);
  let ki = Number.parseFloat(Ki.value);
  let kd = Number.parseFloat(Kd.value);
  let offset = Number.parseFloat(Offset.value);

  Setpoint.value = sp.toFixed(1);
  Kp.value = kp.toFixed(1);
  Ki.value = ki.toFixed(1);
  Kd.value = kd.toFixed(1);
  Offset.value = offset.toFixed(1);
  // then temperature display color may change...
  setTemperature(temperature.innerText);

  asyncAwaitRequest(`/upload?Setpoint=${sp}&Kp=${kp}&Ki=${ki}&Kd=${kd}&Offset=${offset}`);
}

// While the user is entering a value
// ----------------------------------

function digitOnly(event) {
  if (event.keyCode == 13) {
    //        saveThresholds();
    return false;
  }
  return /[\d-\.]/.test(event.key);
}

// ----------------------------------------------------------------------------
// Initialization and handling of the ESP32 control buttons
// ----------------------------------------------------------------------------

function initButtons() {
  btnDefault = document.getElementById("default");
  btnUpload = document.getElementById("upload");
  btnSave = document.getElementById("save");
 // btnReboot = document.getElementById("reboot");
  btnDefault.addEventListener("click", onDefault);
  btnUpload.addEventListener("click", onUpload);
  btnSave.addEventListener("click", onSave);
  //btnReboot.addEventListener("click", onReboot);
}

// Factory reset event manager
// ---------------------------

function onDefault(event) {
  // asynchronous call of the remote routine with the classical method
  //xhrRequest('/reset');

  // for a more modern method, you can instead call this manager:
  asyncAwaitRequest("/default");

  // refreshes all temperature displays
  Setpoint.value = Number.parseFloat(Setpoint.dataset.min).toFixed(1);
  Kp.value = Number.parseFloat(Kp.dataset.max).toFixed(1);
  Ki.value = Number.parseFloat(Ki.dataset.max).toFixed(1);
  Kd.value = Number.parseFloat(Kd.dataset.max).toFixed(1);
  Offset.value = Number.parseFloat(Offset.dataset.max).toFixed(1);
  setTemperature(temperature.innerText);
}

function onUpload(event) {
  let sp = Number.parseFloat(Setpoint.value);
  let kp = Number.parseFloat(Kp.value);
  let ki = Number.parseFloat(Ki.value);
  let kd = Number.parseFloat(Kd.value);
  let offset = Number.parseFloat(Offset.value);
  
  Setpoint.value = sp.toFixed(1);
  Kp.value = kp.toFixed(1);
  Ki.value = ki.toFixed(1);
  Kd.value = kd.toFixed(1);
  Offset.value = offset.toFixed(1);
  // then temperature display color may change...
  setTemperature(temperature.innerText);

  asyncAwaitRequest(`/upload?Setpoint=${sp}&Kp=${kp}&Ki=${ki}&Kd=${kd}&Offset=${offset}`);
}

function onSave(event) {
  let sp = Number.parseFloat(Setpoint.value);
  let kp = Number.parseFloat(Kp.value);
  let ki = Number.parseFloat(Ki.value);
  let kd = Number.parseFloat(Kd.value);
  let offset = Number.parseFloat(Offset.value);
  
  Setpoint.value = sp.toFixed(1);
  Kp.value = kp.toFixed(1);
  Ki.value = ki.toFixed(1);
  Kd.value = kd.toFixed(1);
  Offset.value = offset.toFixed(1);
  // then temperature display color may change...
  setTemperature(temperature.innerText);

  asyncAwaitRequest(`/save?Setpoint=${sp}&Kp=${kp}&Ki=${ki}&Kd=${kd}&Offset=${offset}`);
}

// Event manager for restarting ESP32
// ----------------------------------

function onReboot(event) {
  // sends reboot command to the ESP32
  asyncAwaitRequest("/reboot");
}

// ----------------------------------------------------------------------------
// Initialization and handling of the temperature sensor
// ----------------------------------------------------------------------------

function initProbe() {
  setTemperature(temperature.innerText);
  setInterval(getTemperature, temperatureCaptureTime);
}

// Sending the current temperature reading request
// -----------------------------------------------

function getTemperature() {
  /**
   * a `setTemperature()` callback function is designated in the following
   * to update the temperature display when the ESP32 has transmitted the response.
   */

  // asynchronous call of the remote routine with the classical method
  // xhrRequest('/temp', (temp) => { setTemperature(temp); });

  // for a more modern method, you can instead call this manager:
  asyncAwaitRequest("/temp", (temp) => {
    setTemperature(temp);
  });
}

// Updating the display when the value read on the sensor is received
// ------------------------------------------------------------------

function setTemperature(temp) {
  if (temp == "Error") {
    temperature.innerText = temp;
    unit.style.display = "none";
    screen.className = "error";
  } else {
    let sp = Number.parseFloat(Setpoint.value);

    let t = Number.parseFloat(temp).toFixed(1);

    if (t < sp) {
      screen.className = "cold";
    } else if (t > sp) {
      screen.className = "hot";
    } else {
      screen.className = "";
    }

    temperature.innerText = t;
    unit.style.display = "inline";
  }
}

// -------------------------------------------------------
// AJAX requests
// -------------------------------------------------------

// Using standard vanilla XHR (XMLHttpRequest method)
// @see https://www.w3schools.com/xml/ajax_xmlhttprequest_send.asp

function xhrRequest(path, callback) {
  xhttp.onreadystatechange = function () {
    if (this.readyState == 4 && this.status == 200) {
      // callback is optional!
      typeof callback === "function" && callback(this.responseText);
    }
  };

  xhttp.open("GET", path, true);
  xhttp.send();
}

// Using Async/Await Promises
// @see: https://medium.com/@mattburgess/how-to-get-data-with-javascript-in-2018-f30ba04ad0da

function asyncAwaitRequest(path, callback) {
  (async () => {
    let response = await fetch(path);
    let temp = await response.text();
    // callback is optional!
    typeof callback === "function" && callback(temp);
  })();
}

// ----------------------------------------------------------------------------
// WebSocket Section
// ----------------------------------------------------------------------------

//---------------------------------
var gateway = `ws://${window.location.hostname}/ws`;
var websocket;

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

//window.addEventListener('load', onLoad);

//function onLoad(event) {
    //initWebSocket();
  //  initButton();
//}

// ----------------------------------------------------------------------------
// WebSocket handling
// ----------------------------------------------------------------------------

function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    let data = JSON.parse(event.data);
try 
    {
        if ("status" in data) {
            if (data.status & (1 << 0)) { document.getElementById('led_01').className = "on"; }
            else { document.getElementById('led_01').className = "off"; }
            if (data.status & (1 << 1)) { document.getElementById('led_02').className = "on"; }
            else { document.getElementById('led_02').className = "off"; }
            if (data.status & (1 << 2)) { document.getElementById('led_03').className = "on"; }
            else { document.getElementById('led_03').className = "off"; }
            if (data.status & (1 << 3)) { document.getElementById('led_04').className = "on"; }
            else { document.getElementById('led_04').className = "off"; }
            if (data.status & (1 << 4)) { document.getElementById('led_05').className = "on"; }
            else { document.getElementById('led_05').className = "off"; }
            if (data.status & (1 << 5)) { document.getElementById('led_06').className = "on"; }
            else { document.getElementById('led_06').className = "off"; }
            if (data.status & (1 << 6)) { document.getElementById('led_07').className = "on"; }
            else { document.getElementById('led_07').className = "off"; }
            if (data.status & (1 << 7)) { document.getElementById('led_08').className = "on"; }
            else { document.getElementById('led_08').className = "off"; }
        }
    } 
    catch (error)
    {
        console.error(error);
    }
}

// ----------------------------------------------------------------------------
// Button handling
// ----------------------------------------------------------------------------

function initGPIOButton() {
    document.getElementById('q01').addEventListener('click', onQ01);
    document.getElementById('q02').addEventListener('click', onQ02); 
    document.getElementById('q03').addEventListener('click', onQ03);
    document.getElementById('q04').addEventListener('click', onQ04);  
    document.getElementById('q05').addEventListener('click', onQ05);
    document.getElementById('q06').addEventListener('click', onQ06); 
    document.getElementById('q07').addEventListener('click', onQ07);
    document.getElementById('q08').addEventListener('click', onQ08);  
}

function onQ01(event) {
    websocket.send(JSON.stringify({'action':'q01'}));
}
function onQ02(event) {
    websocket.send(JSON.stringify({'action':'q02'}));
}
function onQ03(event) {
    websocket.send(JSON.stringify({'action':'q03'}));
}
function onQ04(event) {
    websocket.send(JSON.stringify({'action':'q04'}));
}
function onQ05(event) {
    websocket.send(JSON.stringify({'action':'q05'}));
}
function onQ06(event) {
    websocket.send(JSON.stringify({'action':'q06'}));
}

function onQ07(event) {
    websocket.send(JSON.stringify({'action':'q07'}));
}
function onQ08(event) {
    websocket.send(JSON.stringify({'action':'q08'}));
}