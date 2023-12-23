use std::process::Command;

use vergen::EmitBuilder;
fn main() {
    println!("cargo:rerun-if-changed=./src/src_webpack");
    Command::new("rm").arg("./src/webserver/bundle.js").output().expect("failed to execute process");
    
    let output = Command::new("npx").arg("webpack").current_dir("./src_webpack").output().expect("failed to execute process");
    
    println!("status: {}", output.status);
    println!("stdout: {}", String::from_utf8_lossy(&output.stdout));
    println!("stderr: {}", String::from_utf8_lossy(&output.stderr));
    
    assert!(output.status.success());

    embuild::espidf::sysenv::output();
    let _ = EmitBuilder::builder().all_git().emit();
}