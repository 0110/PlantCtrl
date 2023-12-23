export function uploadFile() {
  var file1 = document.getElementById("file1") as HTMLInputElement;
  var loaded_n_total = document.getElementById("loaded_n_total");
  var progressBar = document.getElementById("progressBar") as HTMLProgressElement;
  var status = document.getElementById("status");
  var answer = document.getElementById("answer");

  var file = file1.files[0];
  var ajax = new XMLHttpRequest();


  ajax.upload.addEventListener("progress", event => {
    loaded_n_total.innerHTML = "Uploaded " + event.loaded + " bytes of " + event.total;
    var percent = (event.loaded / event.total) * 100;
    progressBar.value = Math.round(percent);
    status.innerHTML = Math.round(percent) + "%";
    answer.innerHTML = "in progress";
  }, false);
  ajax.addEventListener("load", () => {
    status.innerHTML = ajax.responseText;
    answer.innerHTML = "finished";
    progressBar.value = 0;
  }, false);
  ajax.addEventListener("error", () => {
    status.innerHTML = ajax.responseText;
    answer.innerHTML = "failed";
  }, false);
  ajax.addEventListener("abort", () => {
    status.innerHTML = ajax.responseText;
    answer.innerHTML = "aborted";
  }, false);
  ajax.open("POST", "/ota");
  ajax.send(file);
}

let file1Upload = document.getElementById("file1") as HTMLInputElement;
file1Upload.onchange = uploadFile;
