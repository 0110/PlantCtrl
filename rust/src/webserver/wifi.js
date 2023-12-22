function scanWifi(){
  const req = new XMLHttpRequest();
  req.addEventListener("progress", updateProgress);
  req.addEventListener("load", wifiTransferComplete);
  req.addEventListener("error", transferFailed);
  req.addEventListener("abort", transferCanceled);
  req.open("GET", "/wifiscan");
  req.send();
}

function wifiTransferComplete(evt) {
  console.log("The transfer is complete.");
}

