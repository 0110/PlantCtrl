function saveWifi(){
  document.getElementById("save").disabled = true;
  var wificonfig = {}
  wificonfig.ssid = document.getElementById("ssid").value
  wificonfig.password = document.getElementById("password").value
  var pretty = JSON.stringify(wificonfig, undefined, 4);
  console.log("Sending config " + pretty)

  var ajax = new XMLHttpRequest();
  ajax.upload.addEventListener("progress", progressHandler, false);
  ajax.onreadystatechange = () => {
    _("wifistatus").innerText = ajax.responseText
  };
  ajax.onerror = (evt) => {
    console.log(evt)
    _("wifistatus").innerText = ajax.responseText
    document.getElementById("save").disabled = false;
    alert("Failed to save config see console")
  }
  ajax.open("POST", "/wifisave");
  ajax.send();
}

function scanWifi(){
  document.getElementById("scan").disabled = true;

  var ajax = new XMLHttpRequest();
  ajax.upload.addEventListener("progress", progressHandler, false);
  ajax.responseType = 'json';
  ajax.onreadystatechange = () => {
    if (ajax.readyState === 4) {
      callback(ajax.response);
    }
  };
  ajax.onerror = (evt) => {
    console.log(evt)
    document.getElementById("scan").disabled = false;
    alert("Failed to start see console")
  }
  ajax.open("POST", "/wifiscan");
  ajax.send();
}

function callback(data){
  document.getElementById("scan").disabled = false;

  var ssidlist = document.getElementById("ssidlist")
  ssidlist.innerHTML = ''

  for (ssid of data.ssids) {
    var wi = document.createElement("option");
    wi.value = ssid;
    ssidlist.appendChild(wi);
  }
}

