interface PlantConfig {
  tank_sensor_enabled:boolean,
  tank_full_ml:number,
  tank_warn_percent: number,
  night_lamp_time_start: string,
  night_lamp_time_end: string,
  night_lamp_only_when_dark: boolean,
  plants: {
      target_moisture: number,
      pump_time_s: number,
      pump_cooldown_min: number,
      pump_hour_start: string,
      pump_hour_end: string
    }[]
}

let plants = document.getElementById("plants") as HTMLInputElement;

let fromWrapper = (() => {

  let plantcount = 0;


let tank_full_ml = document.getElementById("tank_full_ml") as HTMLInputElement;
tank_full_ml.onchange = submitForm
let tank_warn_percent = document.getElementById("tank_warn_percent") as HTMLInputElement;
tank_warn_percent.onchange = submitForm
let tank_sensor_enabled = document.getElementById("tank_sensor_enabled") as HTMLInputElement;
tank_sensor_enabled.onchange = submitForm
let night_lamp_only_when_dark = document.getElementById("night_lamp_only_when_dark") as HTMLInputElement;
night_lamp_only_when_dark.onchange = submitForm
let night_lamp_time_start = document.getElementById("night_lamp_time_start") as HTMLInputElement;
night_lamp_time_start.onchange = submitForm
let night_lamp_time_end = document.getElementById("night_lamp_time_end") as HTMLInputElement;
night_lamp_time_end.onchange = submitForm

let json = document.getElementById('json') as HTMLInputElement

function createForm(){
  var current:PlantConfig = {
    tank_sensor_enabled:true,
    tank_full_ml:400,
    tank_warn_percent:50,
    night_lamp_time_start : "18:00",
    night_lamp_time_end : "02:00",
    night_lamp_only_when_dark: true,
    plants :[
      {
        target_moisture: 40,
        pump_time_s: 60,
        pump_cooldown_min: 60,
        pump_hour_start: "10:00",
        pump_hour_end: "18:00"
      }
    ]
  }

  
  for(let i=0;i<current.plants.length;i++){
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
      inputf.onchange = submitForm;
      inputf.type = "number";
      inputf.min = "0";
      inputf.max = "100";
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
      input.onchange = submitForm;
      input.type = "number";
      input.min = "0";
      input.max = "600";
      holder.appendChild(input)

      var text = document.createElement("span");
      holder.appendChild(text)
      text.innerHTML += "Pump Time (s)"
    }
  }
  sync(current);
}

function sync(current:PlantConfig){
  plantcount = current.plants.length
  tank_full_ml.disabled = !current.tank_sensor_enabled;
  tank_warn_percent.disabled = !current.tank_sensor_enabled;

  tank_sensor_enabled.checked = current.tank_sensor_enabled;
  tank_full_ml.value = current.tank_full_ml.toString();
  tank_warn_percent.value = current.tank_warn_percent.toString();
  night_lamp_time_start.value = current.night_lamp_time_start;
  night_lamp_time_end.value = current.night_lamp_time_end;
  
  for(let i=0;i<current.plants.length;i++){
    let plant_target_moisture = document.getElementById("plant_"+i+"_target_moisture") as HTMLInputElement;
    plant_target_moisture.value = current.plants[i].target_moisture.toString();
    let plant_pump_time_s = document.getElementById("plant_"+i+"_pump_time_s") as HTMLInputElement;
    plant_pump_time_s.value = current.plants[i].pump_time_s.toString();
    let plant_pump_cooldown_min = document.getElementById("plant_"+i+"_pump_cooldown_min") as HTMLInputElement;
    plant_pump_cooldown_min.value = current.plants[i].pump_cooldown_min.toString();
    let plant_pump_hour_start = document.getElementById("plant_"+i+"_pump_hour_start") as HTMLInputElement;
    plant_pump_hour_start.value = current.plants[i].pump_hour_start;
    let plant_pump_hour_end = document.getElementById("plant_"+i+"_pump_hour_end") as HTMLInputElement;
    plant_pump_hour_end.value = current.plants[i].pump_hour_end;
  }
}

function submitForm() {
  var current: PlantConfig = {
    tank_sensor_enabled: tank_sensor_enabled.checked,
    tank_full_ml: +tank_full_ml.value,
    tank_warn_percent: +tank_warn_percent.value,
    night_lamp_time_start: night_lamp_time_start.value,
    night_lamp_time_end: night_lamp_time_end.value,
    night_lamp_only_when_dark: night_lamp_only_when_dark.checked,
    plants: []
  }

  for(let i=0;i<plantcount;i++){
    console.log("Adding plant " + i)
    let plant_target_moisture = document.getElementById("plant_"+i+"_target_moisture") as HTMLInputElement;
    let plant_pump_time_s = document.getElementById("plant_"+i+"_pump_time_s") as HTMLInputElement;
    let plant_pump_cooldown_min = document.getElementById("plant_"+i+"_pump_cooldown_min") as HTMLInputElement;
    let plant_pump_hour_start = document.getElementById("plant_"+i+"_pump_hour_start") as HTMLInputElement;
    let plant_pump_hour_end = document.getElementById("plant_"+i+"_pump_hour_end") as HTMLInputElement;

    current.plants[i] = {
      target_moisture : +plant_target_moisture.value,
      pump_time_s: +plant_pump_time_s.value,
      pump_cooldown_min: +plant_pump_cooldown_min.value,
      pump_hour_start: plant_pump_hour_start.value,
      pump_hour_end: plant_pump_hour_end.value

    }
  }
  sync(current);
  console.log(current);

  var pretty = JSON.stringify(current, undefined, 4);
  json.value = pretty;
}

let createDocumentBtn = document.getElementById("create") as HTMLButtonElement
if(createDocumentBtn){
  createDocumentBtn.onclick = createForm;
}
let submitFormBtn = document.getElementById("submit") as HTMLButtonElement
if(submitFormBtn){
  submitFormBtn.onclick = submitForm;
}



})
if(plants){
  fromWrapper()
}

