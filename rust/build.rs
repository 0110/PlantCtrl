use vergen::EmitBuilder;
fn main() {
    embuild::espidf::sysenv::output();
    let _ = EmitBuilder::builder().all_git().emit();
}