use std::process::Command;

#[test]
fn version() {
    let output = Command::new(env!("CARGO_BIN_EXE_urdf-viz"))
        .arg("--version")
        .output()
        .unwrap();
    assert!(output.status.success());
    assert_eq!(
        String::from_utf8(output.stdout).unwrap().trim(),
        format!("urdf-viz {}", env!("CARGO_PKG_VERSION"))
    );
}
