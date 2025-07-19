use klippy_rust::reactor::Reactor;
use klippy_rust::Printer;

#[test]
fn test_printer_new() {
    let reactor = Reactor::new();
    let _printer = Printer::new(reactor);
}

#[test]
fn test_printer_shutdown() {
    let reactor = Reactor::new();
    let mut printer = Printer::new(reactor);
    printer.invoke_shutdown("test");
    assert_eq!(printer.is_shutdown(), true);
}

#[test]
fn test_printer_exit() {
    let reactor = Reactor::new();
    let mut printer = Printer::new(reactor);
    printer.request_exit("test");
    assert_eq!(printer.get_run_result(), Some("test".to_string()));
}
