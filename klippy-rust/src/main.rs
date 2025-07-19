use clap::Parser;
use klippy_rust::reactor::Reactor;
use klippy_rust::Printer;
use log::{info, warn};

/// Klippy host software
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Config file
    #[arg()]
    config_file: String,

    /// read commands from file instead of from tty port
    #[arg(short, long)]
    debuginput: Option<String>,

    /// input tty name
    #[arg(short = 'I', long, default_value = "/tmp/printer")]
    input_tty: String,

    /// api server unix domain socket filename
    #[arg(short, long)]
    api_server: Option<String>,

    /// write log to file instead of stderr
    #[arg(short, long)]
    logfile: Option<String>,

    /// enable debug messages
    #[arg(short, long)]
    verbose: bool,

    /// write output to file instead of to serial port
    #[arg(short, long)]
    debugoutput: Option<String>,

    /// file to read for mcu protocol dictionary
    #[arg(short, long)]
    dictionary: Option<String>,

    /// perform an import module test
    #[arg(long)]
    import_test: bool,
}

fn main() {
    let args = Args::parse();

    let mut builder = env_logger::Builder::from_default_env();
    if args.verbose {
        builder.filter(None, log::LevelFilter::Debug);
    } else {
        builder.filter(None, log::LevelFilter::Info);
    }
    if let Some(ref logfile) = args.logfile {
        let target = Box::new(std::fs::File::create(logfile).expect("Can't create logfile"));
        builder.target(env_logger::Target::Pipe(target));
    }
    builder.init();

    info!("Starting Klippy...");

    if !args.debugoutput.is_some() && args.logfile.is_none() {
        warn!("No log file specified! Severe timing issues may result!");
    }

    loop {
        info!("Restarting printer");
        let main_reactor = Reactor::new();
        let mut printer = Printer::new(main_reactor);
        let res = printer.run();
        if res == Some("exit".to_string()) || res == Some("error_exit".to_string()) {
            break;
        }
    }
}
