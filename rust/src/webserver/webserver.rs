//offer ota and config mode

use std::{vec, sync::{Mutex, Arc}};

use embedded_svc::http::Method;
use esp_idf_svc::http::server::EspHttpServer;
use esp_ota::OtaUpdate;

use crate::plant_hal::{PlantCtrlBoard, PlantCtrlBoardInteraction};

pub fn httpd_initial(board_access:Arc<Mutex<PlantCtrlBoard<'static>>>) -> Box<EspHttpServer<'static>> {
    let mut server = shared();
    server.fn_handler("/",Method::Get, move |request| {
        let mut response = request.into_ok_response()?;
        response.write(include_bytes!("initial_config.html"))?;
        return Ok(())
    }).unwrap();

    server.fn_handler("/wifiscan",Method::Get,  move |request| {
        let mut response = request.into_ok_response()?;
        let mut board = board_access.lock().unwrap();
        match board.wifi_scan()  {
            Err(error) => {
                response.write(format!("Error scanning wifi: {}", error).as_bytes())?;
            },
            Ok(scan_result) => {
                println!("Scan result is {:?}", scan_result);
                response.write("{ ssids:[".as_bytes())?;
                let mut first = true;
                for ap in scan_result.iter(){
                    if !first {
                        response.write(",".as_bytes())?;
                    }
                    response.write(ap.ssid.as_bytes())?;
                    first = false;
                }
                response.write("]".as_bytes())?;
            },
        }
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
