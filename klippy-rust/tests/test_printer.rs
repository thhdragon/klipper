use klippy_rust::reactor::Reactor;
use klippy_rust::Printer;

#[test]
fn test_printer_new() {
    let reactor = Reactor::new();
    let _printer = Printer::new(reactor);
}
