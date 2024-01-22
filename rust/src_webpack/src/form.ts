interface PlantConfig {
  mqtt_url: string,
  base_topic: string,
  tank_sensor_enabled: boolean,
  tank_allow_pumping_if_sensor_error: boolean,
  tank_useable_ml: number,
  tank_warn_percent: number,
  tank_empty_mv: number,
  tank_full_mv: number,
  night_lamp_hour_start: number,
  night_lamp_hour_end: number,
  night_lamp_only_when_dark: boolean,
  max_consecutive_pump_count: number,
  
  plants: {
    mode: string,
    target_moisture: number,
    pump_time_s: number,
    pump_cooldown_min: number,
    pump_hour_start: number,
    pump_hour_end: number
  }[]
}

let plants = document.getElementById("plants") as HTMLInputElement;

let fromWrapper = (() => {

  let plantcount = 0;

  function addTimeOptions(select: HTMLSelectElement) {
    for (let i = 0; i < 24; i++) {
      let option = document.createElement("option");
      option.innerText = i.toString();
      select.appendChild(option);
    }
  }

  
  let mqtt_url = document.getElementById("mqtt_url") as HTMLInputElement;
  mqtt_url.onchange = updateJson
  let base_topic = document.getElementById("base_topic") as HTMLInputElement;
  base_topic.onchange = updateJson
  let max_consecutive_pump_count = document.getElementById("max_consecutive_pump_count") as HTMLInputElement;
  max_consecutive_pump_count.onchange = updateJson

  let tank_useable_ml = document.getElementById("tank_useable_ml") as HTMLInputElement;
  tank_useable_ml.onchange = updateJson
  let tank_empty_mv = document.getElementById("tank_empty_mv") as HTMLInputElement;
  tank_empty_mv.onchange = updateJson
  let tank_full_mv = document.getElementById("tank_full_mv") as HTMLInputElement;
  tank_full_mv.onchange = updateJson
  let tank_warn_percent = document.getElementById("tank_warn_percent") as HTMLInputElement;
  tank_warn_percent.onchange = updateJson
  let tank_sensor_enabled = document.getElementById("tank_sensor_enabled") as HTMLInputElement;
  tank_sensor_enabled.onchange = updateJson
  let tank_allow_pumping_if_sensor_error = document.getElementById("tank_allow_pumping_if_sensor_error") as HTMLInputElement;
  tank_allow_pumping_if_sensor_error.onchange = updateJson
  let night_lamp_only_when_dark = document.getElementById("night_lamp_only_when_dark") as HTMLInputElement;
  night_lamp_only_when_dark.onchange = updateJson
  let night_lamp_time_start = document.getElementById("night_lamp_time_start") as HTMLSelectElement;
  night_lamp_time_start.onchange = updateJson
  addTimeOptions(night_lamp_time_start);
  let night_lamp_time_end = document.getElementById("night_lamp_time_end") as HTMLSelectElement;
  night_lamp_time_end.onchange = updateJson
  addTimeOptions(night_lamp_time_end);

  let json = document.getElementById('json') as HTMLInputElement

  function createForm(current: PlantConfig) {
    for (let i = 0; i < current.plants.length; i++) {
      let plant = document.createElement("div");
      plants.appendChild(plant);
      let header = document.createElement("h4");
      header.textContent = "Plant " + (i + 1);
      plant.appendChild(header);

      {
        let holder = document.createElement("div");
        plant.appendChild(holder);
        let inputf = document.createElement("select");
        inputf.id = "plant_" + i + "_mode";
        inputf.onchange = updateJson;
        holder.appendChild(inputf)

        let optionOff = document.createElement("option");
        optionOff.value = "OFF";
        optionOff.innerText = "Off";
        inputf.appendChild(optionOff);

        let optionTargetMoisture = document.createElement("option");
        optionTargetMoisture.value = "TargetMoisture";
        optionTargetMoisture.innerText = "Target Moisture";
        inputf.appendChild(optionTargetMoisture);

        let optionTimerOnly = document.createElement("option");
        optionTimerOnly.value = "TimerOnly";
        optionTimerOnly.innerText = "Timer";
        inputf.appendChild(optionTimerOnly);

        let text = document.createElement("span");
        holder.appendChild(text)
        text.innerHTML += "Mode"
      }
      {
        let holder = document.createElement("div");
        plant.appendChild(holder);
        let inputf = document.createElement("input");
        inputf.id = "plant_" + i + "_target_moisture";
        inputf.onchange = updateJson;
        inputf.type = "number";
        inputf.min = "0";
        inputf.max = "100";
        holder.appendChild(inputf)

        let text = document.createElement("span");
        holder.appendChild(text)
        text.innerHTML += "Target Moisture"
      }
      {
        let holder = document.createElement("div");
        plant.appendChild(holder);
        let input = document.createElement("input");
        input.id = "plant_" + i + "_pump_time_s";
        input.onchange = updateJson;
        input.type = "number";
        input.min = "0";
        input.max = "600";
        holder.appendChild(input)

        let text = document.createElement("span");
        holder.appendChild(text)
        text.innerHTML += "Pump Time (s)"
      }
      {
        let holder = document.createElement("div");
        plant.appendChild(holder);
        let input = document.createElement("input");
        input.id = "plant_" + i + "_pump_cooldown_min";
        input.onchange = updateJson;
        input.type = "number";
        input.min = "0";
        input.max = "600";
        holder.appendChild(input)

        let text = document.createElement("span");
        holder.appendChild(text)
        text.innerHTML += "Pump Cooldown (m)"
      }
      {
        let holder = document.createElement("div");
        plant.appendChild(holder);
        let input = document.createElement("select");
        input.id = "plant_" + i + "_pump_hour_start";
        addTimeOptions(input);
        input.onchange = updateJson;
        holder.appendChild(input)

        let text = document.createElement("span");
        holder.appendChild(text)
        text.innerHTML += "Pump Hour Start"
      }
      {
        let holder = document.createElement("div");
        plant.appendChild(holder);
        let input = document.createElement("select");
        input.id = "plant_" + i + "_pump_hour_end";
        addTimeOptions(input);
        input.onchange = updateJson;
        holder.appendChild(input)

        let text = document.createElement("span");
        holder.appendChild(text)
        text.innerHTML += "Pump Hour End"
      }
    }
    sync(current);
  }

  function sync(current: PlantConfig) {
    plantcount = current.plants.length
    mqtt_url.value = current.mqtt_url;
    base_topic.value = current.base_topic;
    max_consecutive_pump_count.value = current.max_consecutive_pump_count.toString();
    
    tank_useable_ml.disabled = !current.tank_sensor_enabled;
    tank_warn_percent.disabled = !current.tank_sensor_enabled;
    tank_sensor_enabled.checked = current.tank_sensor_enabled;
    tank_allow_pumping_if_sensor_error.checked = current.tank_allow_pumping_if_sensor_error;
    tank_useable_ml.value = current.tank_useable_ml.toString();
    tank_warn_percent.value = current.tank_warn_percent.toString();
    tank_empty_mv.value = current.tank_empty_mv.toString();
    tank_full_mv.value = current.tank_full_mv.toString();

    night_lamp_time_start.value = current.night_lamp_hour_start.toString();
    night_lamp_time_end.value = current.night_lamp_hour_end.toString();

    for (let i = 0; i < current.plants.length; i++) {
      let plant_mode = document.getElementById("plant_" + i + "_mode") as HTMLSelectElement;
      plant_mode.value = current.plants[i].mode;
      let plant_target_moisture = document.getElementById("plant_" + i + "_target_moisture") as HTMLInputElement;
      plant_target_moisture.value = current.plants[i].target_moisture.toString();
      let plant_pump_time_s = document.getElementById("plant_" + i + "_pump_time_s") as HTMLInputElement;
      plant_pump_time_s.value = current.plants[i].pump_time_s.toString();
      let plant_pump_cooldown_min = document.getElementById("plant_" + i + "_pump_cooldown_min") as HTMLInputElement;
      plant_pump_cooldown_min.value = current.plants[i].pump_cooldown_min.toString();
      let plant_pump_hour_start = document.getElementById("plant_" + i + "_pump_hour_start") as HTMLInputElement;
      plant_pump_hour_start.value = current.plants[i].pump_hour_start.toString();
      let plant_pump_hour_end = document.getElementById("plant_" + i + "_pump_hour_end") as HTMLInputElement;
      plant_pump_hour_end.value = current.plants[i].pump_hour_end.toString();
    }
  }

  function updateJson() {
    var current: PlantConfig = {
      max_consecutive_pump_count: +max_consecutive_pump_count.value,
      mqtt_url: mqtt_url.value,
      base_topic: base_topic.value,
      tank_allow_pumping_if_sensor_error: tank_allow_pumping_if_sensor_error.checked,
      tank_sensor_enabled: tank_sensor_enabled.checked,
      tank_useable_ml: +tank_useable_ml.value,
      tank_warn_percent: +tank_warn_percent.value,
      tank_empty_mv: +tank_empty_mv.value,
      tank_full_mv: +tank_full_mv.value,
      night_lamp_hour_start: +night_lamp_time_start.value,
      night_lamp_hour_end: +night_lamp_time_end.value,
      night_lamp_only_when_dark: night_lamp_only_when_dark.checked,
      plants: []
    }

    for (let i = 0; i < plantcount; i++) {
      console.log("Adding plant " + i)
      let plant_mode = document.getElementById("plant_" + i + "_mode") as HTMLSelectElement;
      let plant_target_moisture = document.getElementById("plant_" + i + "_target_moisture") as HTMLInputElement;
      let plant_pump_time_s = document.getElementById("plant_" + i + "_pump_time_s") as HTMLInputElement;
      let plant_pump_cooldown_min = document.getElementById("plant_" + i + "_pump_cooldown_min") as HTMLInputElement;
      let plant_pump_hour_start = document.getElementById("plant_" + i + "_pump_hour_start") as HTMLInputElement;
      let plant_pump_hour_end = document.getElementById("plant_" + i + "_pump_hour_end") as HTMLInputElement;

      current.plants[i] = {
        mode: plant_mode.value,
        target_moisture: +plant_target_moisture.value,
        pump_time_s: +plant_pump_time_s.value,
        pump_cooldown_min: +plant_pump_cooldown_min.value,
        pump_hour_start: +plant_pump_hour_start.value,
        pump_hour_end: +plant_pump_hour_end.value

      }
    }
    sync(current);
    console.log(current);

    var pretty = JSON.stringify(current, undefined, 1);
    json.value = pretty;
  }

  let submitFormBtn = document.getElementById("submit") as HTMLButtonElement
  let submit_status = document.getElementById("submit_status")
  
  if (submitFormBtn) {
    
    submitFormBtn.onclick = function (){
      updateJson()
      fetch("/set_config", {
        method :"POST",
        body: json.value
      })
      .then(response => response.text())
      .then(text => submit_status.innerText = text)

    };

  }

  fetch("/get_config")
    .then(response => response.json())
    .then(json => { createForm(json as PlantConfig); }
    )
})
if (plants) {
  fromWrapper()
}
