var plantcount = 1;

function createForm(){
  var current = {}
  current.tank_sensor_enabled = true;
  current.tank_full_ml = 400;
  current.tank_warn_percent = 200;
  current.night_lamp_time_start = "18:00";
  current.night_lamp_time_end = "02:00";
  current.night_lamp_only_when_dark = true;
  current.plants = [
    {
      target_moisture: 40,
      pump_time_s:60
    }

  ]
  current.plantcount = 1;

  plantcount = current.plantcount;
  

  for(i=0;i<plantcount;i++){
    var plant = document.createElement("div");
    plants.appendChild(plant);

    var header = document.createElement("h4");
    header.textContent = "Plant " + (i+1);
    plant.appendChild(header);

    {
      var holder = document.createElement("div");
      plant.appendChild(holder);
      var inputf = document.createElement("input");
      inputf.id = "plant_"+i+"_target_moisture";
      inputf.onchange = function() {submitForm()};
      inputf.type = "number";
      inputf.min = 0;
      inputf.max = 100;
      holder.appendChild(inputf)

      var text = document.createElement("span");
      holder.appendChild(text)
      text.innerHTML += "Target Moisture"
    }
    {
      var holder = document.createElement("div");
      plant.appendChild(holder);
      var input = document.createElement("input");
      input.id = "plant_"+i+"_pump_time_s";
      input.onchange = function() {submitForm()};
      input.type = "number";
      input.min = 0;
      input.max = 600;
      holder.appendChild(input)

      var text = document.createElement("span");
      holder.appendChild(text)
      text.innerHTML += "Pump Time (s)"
    }

  }
  

  sync(current);
}

function sync(current){
  document.getElementById("tank_full_ml").disabled = !current.tank_sensor_enabled;
  document.getElementById("tank_warn_percent").disabled = !current.tank_sensor_enabled;

  document.getElementById("tank_sensor_enabled").checked = current.tank_sensor_enabled;
  document.getElementById("tank_full_ml").value = current.tank_full_ml;
  document.getElementById("tank_warn_percent").value = current.tank_warn_percent;
  document.getElementById("night_lamp_time_start").value = current.night_lamp_time_start;
  document.getElementById("night_lamp_time_end").value = current.night_lamp_time_end;
  document.getElementById("tank_warn_percent").value = current.tank_warn_percent;

  for(i=0;i<plantcount;i++){
    document.getElementById("plant_"+i+"_target_moisture").value = current.plants[i].target_moisture;
    document.getElementById("plant_"+i+"_pump_time_s").value = current.plants[i].pump_time_s;
  }
}

function submitForm() {
  var current = {}
  current.plantcount = plantcount;

  current.tank_sensor_enabled = document.getElementById("tank_sensor_enabled").checked;
  current.tank_full_ml = document.getElementById("tank_full_ml").value;
  current.tank_warn_percent = document.getElementById("tank_warn_percent").value;
  current.night_lamp_time_start = document.getElementById("night_lamp_time_start").value;
  current.night_lamp_time_end = document.getElementById("night_lamp_time_end").value;
  current.night_lamp_only_when_dark = document.getElementById("night_lamp_only_when_dark").checked;

  current.plants = []
  for(i=0;i<plantcount;i++){
    console.log("Adding plant " + i)
    current.plants[i] = {}
    current.plants[i].target_moisture = document.getElementById("plant_"+i+"_target_moisture").value;
    current.plants[i].pump_time_s = document.getElementById("plant_"+i+"_pump_time_s").value;
  }
  
  sync(current);
  console.log(current);

  var pretty = JSON.stringify(current, undefined, 4);
  document.getElementById('json').value = pretty;
}
