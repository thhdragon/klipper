use klippy_rust::webhooks;
use klippy_rust::Printer;
use klippy_rust::reactor::Reactor;

#[test]
fn test_webhooks_add_early_printer_objects() {
    let reactor = Reactor::new();
    let mut printer = Printer::new(reactor);
    webhooks::add_early_printer_objects(&mut printer);
}
