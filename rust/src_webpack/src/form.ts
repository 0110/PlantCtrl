interface PlantConfig {
  tank_sensor_enabled: boolean,
  tank_full_ml: number,
  tank_warn_percent: number,
  night_lamp_hour_start: number,
  night_lamp_hour_end: number,
  night_lamp_only_when_dark: boolean,
  plants: {
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

  let tank_full_ml = document.getElementById("tank_full_ml") as HTMLInputElement;
  tank_full_ml.onchange = updateJson
  let tank_warn_percent = document.getElementById("tank_warn_percent") as HTMLInputElement;
  tank_warn_percent.onchange = updateJson
  let tank_sensor_enabled = document.getElementById("tank_sensor_enabled") as HTMLInputElement;
  tank_sensor_enabled.onchange = updateJson
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
    tank_full_ml.disabled = !current.tank_sensor_enabled;
    tank_warn_percent.disabled = !current.tank_sensor_enabled;

    tank_sensor_enabled.checked = current.tank_sensor_enabled;
    tank_full_ml.value = current.tank_full_ml.toString();
    tank_warn_percent.value = current.tank_warn_percent.toString();
    night_lamp_time_start.value = current.night_lamp_hour_start.toString();
    night_lamp_time_end.value = current.night_lamp_hour_end.toString();

    for (let i = 0; i < current.plants.length; i++) {
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
      tank_sensor_enabled: tank_sensor_enabled.checked,
      tank_full_ml: +tank_full_ml.value,
      tank_warn_percent: +tank_warn_percent.value,
      night_lamp_hour_start: +night_lamp_time_start.value,
      night_lamp_hour_end: +night_lamp_time_end.value,
      night_lamp_only_when_dark: night_lamp_only_when_dark.checked,
      plants: []
    }

    for (let i = 0; i < plantcount; i++) {
      console.log("Adding plant " + i)
      let plant_target_moisture = document.getElementById("plant_" + i + "_target_moisture") as HTMLInputElement;
      let plant_pump_time_s = document.getElementById("plant_" + i + "_pump_time_s") as HTMLInputElement;
      let plant_pump_cooldown_min = document.getElementById("plant_" + i + "_pump_cooldown_min") as HTMLInputElement;
      let plant_pump_hour_start = document.getElementById("plant_" + i + "_pump_hour_start") as HTMLInputElement;
      let plant_pump_hour_end = document.getElementById("plant_" + i + "_pump_hour_end") as HTMLInputElement;

      current.plants[i] = {
        target_moisture: +plant_target_moisture.value,
        pump_time_s: +plant_pump_time_s.value,
        pump_cooldown_min: +plant_pump_cooldown_min.value,
        pump_hour_start: +plant_pump_hour_start.value,
        pump_hour_end: +plant_pump_hour_end.value

      }
    }
    sync(current);
    console.log(current);

    var pretty = JSON.stringify(current, undefined, 4);
    json.value = pretty;
  }

  let submitFormBtn = document.getElementById("submit") as HTMLButtonElement
  if (submitFormBtn) {
    submitFormBtn.onclick = function (){
      updateJson()
      fetch("/set_config", {
        method :"POST",
        body: json.value
      })
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

