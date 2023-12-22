//offer ota and config mode

use std::sync::{Mutex, Arc};

use embedded_svc::http::Method;
use esp_idf_svc::http::server::EspHttpServer;
use esp_ota::OtaUpdate;
use heapless::String;
use serde::Serialize;

use crate::{plant_hal::{PlantCtrlBoard, PlantCtrlBoardInteraction}, config::WifiConfig};

#[derive(Serialize)]
#[derive(Debug)]
struct SSIDList<'a> {
    ssids: Vec<&'a String<32>>
}

pub fn httpd_initial(board_access:&Arc<Mutex<PlantCtrlBoard<'static>>>) -> Box<EspHttpServer<'static>> {
    let mut server = shared();
    server.fn_handler("/",Method::Get, move |request| {
        let mut response = request.into_ok_response()?;
        response.write(include_bytes!("initial_config.html"))?;
        return Ok(())
    }).unwrap();
    server
        .fn_handler("/wifi.js",Method::Get, |request| {
            let mut response = request.into_ok_response()?;
            response.write(include_bytes!("wifi.js"))?;
            return Ok(())
        }).unwrap();
    
    server.fn_handler("/wifiscan",Method::Post,  move |request| {
        let mut response = request.into_ok_response()?;
        let mut board = board_access.lock().unwrap();
        match board.wifi_scan()  {
            Err(error) => {
                response.write(format!("Error scanning wifi: {}", error).as_bytes())?;
            },
            Ok(scan_result) => {
                let mut ssids: Vec<&String<32>> = Vec::new();
                scan_result.iter().for_each(|s| 
                    ssids.push(&s.ssid)
                );
                response.write( serde_json::to_string(
                    &SSIDList{ssids}
                )?.as_bytes())?;
            },
        }
        return Ok(())
    }).unwrap();

    server.fn_handler("/wifisave",Method::Post,  move |mut request| {
        let mut buf = [0_u8;2048];
        let read = request.read(&mut buf);
        if read.is_err(){
            let error_text = read.unwrap_err().to_string();
            request.into_status_response(500)?.write(error_text.as_bytes())?;
            return Ok(());
        }
        let actual_data = &buf[0..read.unwrap()];
        let wifi_config: Result<WifiConfig, serde_json::Error> = serde_json::from_slice(actual_data);
        if wifi_config.is_err(){
            let error_text = wifi_config.unwrap_err().to_string();
            request.into_status_response(500)?.write(error_text.as_bytes())?;
            return Ok(());
        }
        let mut board = board_access.lock().unwrap();
        board.set_wifi(&wifi_config.unwrap());
        let mut response = request.into_status_response(202)?;
        response.write("saved".as_bytes());
        return Ok(())
    }).unwrap();

    
    return server
}

pub fn httpd(board:&mut Box<PlantCtrlBoard<'static>>) -> Box<EspHttpServer<'static>> {
    let mut server = shared();

    server
    .fn_handler("/",Method::Get, move |request| {
        let mut response = request.into_ok_response()?;
        response.write(include_bytes!("config.html"))?;
        return Ok(())
    }).unwrap();

    return server;

}

pub fn shared() -> Box<EspHttpServer<'static>> {
    let mut server = Box::new(EspHttpServer::new(&Default::default()).unwrap());

    server
        .fn_handler("/version",Method::Get,  |request| {
            let mut response = request.into_ok_response()?;
            response.write(env!("VERGEN_GIT_DESCRIBE").as_bytes())?;
            return Ok(())
        }).unwrap();
    server
        .fn_handler("/ota.js",Method::Get, |request| {
            let mut response = request.into_ok_response()?;
            response.write(include_bytes!("ota.js"))?;
            return Ok(())
        }).unwrap();
    server
        .fn_handler("/ota", Method::Post,  |mut request| {
            let ota = OtaUpdate::begin();
            if ota.is_err(){
                let error_text = ota.unwrap_err().to_string();
                request.into_status_response(500)?.write(error_text.as_bytes())?;
                return Ok(());
            }
            let mut ota = ota.unwrap();
            println!("start ota");
            
            //having a larger buffer is not really faster, requires more stack and prevents the progress bar from working ;)
            const BUFFER_SIZE:usize = 512;
            let mut buffer :[u8;BUFFER_SIZE] = [0;BUFFER_SIZE];
            let mut total_read: usize = 0;
            loop {
                let read = request.read(&mut buffer).unwrap();
                total_read += read;
                println!("received {read} bytes ota {total_read}");
                let to_write =  & buffer[0 .. read];

                
                let write_result = ota.write(to_write);
                if write_result.is_err(){
                    let error_text = write_result.unwrap_err().to_string();
                    request.into_status_response(500)?.write(error_text.as_bytes())?;
                    return Ok(());
                }
                println!("wrote {read} bytes ota {total_read}");
                if read == 0 {
                    break;
                }
            }
            println!("finish ota");
            let partition = ota.raw_partition();
            println!("finalizing and changing boot partition to {partition:?}");

            let finalizer = ota.finalize();
            if finalizer.is_err(){
                let error_text = finalizer.err().unwrap().to_string();
                request.into_status_response(500)?.write(error_text.as_bytes())?;
                return Ok(());
            }
            let mut finalizer = finalizer.unwrap();
            
            println!("changing boot partition");
            finalizer.set_as_boot_partition().unwrap();
            finalizer.restart();
        }).unwrap();
    return server;
}
