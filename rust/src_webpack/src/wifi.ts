interface WifiConfig {
  ssid: string;
  password: string;
}

interface SSIDList {
  ssids : [string]
}

export function saveWifi(){
  var saveButton = (document.getElementById("save") as HTMLButtonElement);
  saveButton.disabled = true;
  var ssid = (document.getElementById("ssid") as HTMLInputElement).value
  var password = (document.getElementById("password") as HTMLInputElement).value
  
  var wifistatus = document.getElementById("wifistatus")

  var wificonfig:WifiConfig = {ssid, password}
  var pretty = JSON.stringify(wificonfig, undefined, 4);
  console.log("Sending config " + pretty)

  var ajax = new XMLHttpRequest();
  ajax.onreadystatechange = () => {
    wifistatus.innerText = ajax.responseText
  };
  ajax.onerror = (evt) => {
    console.log(evt)
    wifistatus.innerText = ajax.responseText
    saveButton.disabled = false;
    alert("Failed to save config see console")
  }
  ajax.open("POST", "/wifisave");
  ajax.send(pretty);
}

export function scanWifi(){
  var scanButton = (document.getElementById("scan") as HTMLButtonElement);
  scanButton.disabled = true;

  var ajax = new XMLHttpRequest();
  ajax.responseType = 'json';
  ajax.onreadystatechange = () => {
    if (ajax.readyState === 4) {
      callback(ajax.response);
    }
  };
  ajax.onerror = (evt) => {
    console.log(evt)
    scanButton.disabled = false;
    alert("Failed to start see console")
  }
  ajax.open("POST", "/wifiscan");
  ajax.send();
}

function test(){
  var testButton = (document.getElementById("test") as HTMLButtonElement);
  testButton.disabled = true;

  var ajax = new XMLHttpRequest();
  ajax.responseType = 'json';
  ajax.onerror = (evt) => {
    console.log(evt)
    testButton.disabled = false;
    alert("Failed to start see console")
  }
  ajax.open("POST", "/boardtest");
  ajax.send();
}

function callback(data:SSIDList){
  var ssidlist = document.getElementById("ssidlist")
  ssidlist.innerHTML = ''

  for (var ssid of data.ssids) {
    var wi = document.createElement("option");
    wi.value = ssid;
    ssidlist.appendChild(wi);
  }
}

let testBtn = document.getElementById("test") as HTMLButtonElement;
if(testBtn){
  testBtn.onclick = test;
}
let scanWifiBtn = document.getElementById("scan") as HTMLButtonElement;
if(scanWifiBtn){
  scanWifiBtn.onclick = scanWifi;
}
let saveWifiBtn = document.getElementById("save") as HTMLButtonElement;
if(saveWifiBtn){
  saveWifiBtn.onclick = saveWifi;
}