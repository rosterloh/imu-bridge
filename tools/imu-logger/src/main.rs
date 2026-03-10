use std::env;
use std::error::Error;
use std::fmt;
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use zenoh::{Config, Wait};

const DEFAULT_CONNECT: &str = "tcp/127.0.0.1:7447";
const DEFAULT_KEYEXPR: &str = "imu/reading";
const DEFAULT_OUTPUT: &str = "data/imu-readings.csv";
const CSV_HEADER: &str = "timestamp_us,accel_x_mg,accel_y_mg,accel_z_mg,gyro_x_mdps,gyro_y_mdps,gyro_z_mdps,temperature_c";

type AppError = Box<dyn Error + Send + Sync>;

fn main() -> Result<(), AppError> {
    let args = Args::parse(env::args().skip(1))?;
    let mut writer = CsvLogger::open(&args.output)?;

    let mut config = Config::default();
    config.insert_json5("mode", "\"client\"")?;
    config.insert_json5("connect/endpoints", &format!("[\"{}\"]", args.connect))?;
    config.insert_json5("scouting/multicast/enabled", "false")?;

    println!(
        "Connecting to {} and subscribing to {}",
        args.connect, args.keyexpr
    );

    let session = zenoh::open(config).wait()?;
    let subscriber = session.declare_subscriber(args.keyexpr.as_str()).wait()?;

    println!("Logging IMU samples to {}", args.output.display());

    loop {
        let sample = subscriber.recv()?;
        let payload = sample.payload().try_to_string()?;
        let reading = Reading::parse(payload.as_ref())?;
        writer.write(&reading)?;
    }
}

struct Args {
    connect: String,
    keyexpr: String,
    output: PathBuf,
}

impl Args {
    fn parse<I>(mut args: I) -> Result<Self, AppError>
    where
        I: Iterator<Item = String>,
    {
        let mut connect = DEFAULT_CONNECT.to_owned();
        let mut keyexpr = DEFAULT_KEYEXPR.to_owned();
        let mut output = PathBuf::from(DEFAULT_OUTPUT);

        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--connect" => connect = next_value(&mut args, "--connect")?,
                "--keyexpr" => keyexpr = next_value(&mut args, "--keyexpr")?,
                "--output" => output = PathBuf::from(next_value(&mut args, "--output")?),
                "-h" | "--help" => {
                    print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown argument: {arg}\n\n{}", usage_text()).into()),
            }
        }

        Ok(Self {
            connect,
            keyexpr,
            output,
        })
    }
}

fn next_value<I>(args: &mut I, flag: &str) -> Result<String, AppError>
where
    I: Iterator<Item = String>,
{
    args.next()
        .ok_or_else(|| format!("missing value for {flag}").into())
}

fn print_usage() {
    println!("{}", usage_text());
}

fn usage_text() -> &'static str {
    "Usage: imu-logger [--connect tcp/127.0.0.1:7447] [--keyexpr imu/reading] [--output data/imu-readings.csv]"
}

struct CsvLogger {
    writer: BufWriter<File>,
}

impl CsvLogger {
    fn open(path: &Path) -> Result<Self, AppError> {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }

        let file_exists = path.exists();
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        let mut writer = BufWriter::new(file);

        if !file_exists {
            writeln!(writer, "{CSV_HEADER}")?;
            writer.flush()?;
        }

        Ok(Self { writer })
    }

    fn write(&mut self, reading: &Reading) -> Result<(), AppError> {
        writeln!(
            self.writer,
            "{},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2}",
            reading.timestamp_us,
            reading.acceleration_mg[0],
            reading.acceleration_mg[1],
            reading.acceleration_mg[2],
            reading.angular_rate_mdps[0],
            reading.angular_rate_mdps[1],
            reading.angular_rate_mdps[2],
            reading.temperature_c
        )?;
        self.writer.flush()?;
        Ok(())
    }
}

struct Reading {
    timestamp_us: u64,
    acceleration_mg: [f32; 3],
    angular_rate_mdps: [f32; 3],
    temperature_c: f32,
}

impl Reading {
    fn parse(payload: &str) -> Result<Self, ParseReadingError> {
        let mut parts = payload.split_whitespace();

        let timestamp_us = parse_u64_field(parts.next(), "ts_us=")?;
        let acceleration_mg = parse_vec3_field(parts.next(), "accel_mg=")?;
        let angular_rate_mdps = parse_vec3_field(parts.next(), "gyro_mdps=")?;
        let temperature_c = parse_f32_field(parts.next(), "temp_c=")?;

        if let Some(extra) = parts.next() {
            return Err(ParseReadingError::InvalidFormat(format!(
                "unexpected trailing field: {extra}"
            )));
        }

        Ok(Self {
            timestamp_us,
            acceleration_mg,
            angular_rate_mdps,
            temperature_c,
        })
    }
}

fn parse_u64_field(field: Option<&str>, prefix: &str) -> Result<u64, ParseReadingError> {
    let value = require_prefix(field, prefix)?;
    value.parse().map_err(|_| ParseReadingError::InvalidNumber {
        field: prefix.trim_end_matches('=').to_owned(),
        value: value.to_owned(),
    })
}

fn parse_f32_field(field: Option<&str>, prefix: &str) -> Result<f32, ParseReadingError> {
    let value = require_prefix(field, prefix)?;
    value.parse().map_err(|_| ParseReadingError::InvalidNumber {
        field: prefix.trim_end_matches('=').to_owned(),
        value: value.to_owned(),
    })
}

fn parse_vec3_field(field: Option<&str>, prefix: &str) -> Result<[f32; 3], ParseReadingError> {
    let value = require_prefix(field, prefix)?;
    let mut parts = value.split(',');

    let x = parse_vec_component(parts.next(), prefix)?;
    let y = parse_vec_component(parts.next(), prefix)?;
    let z = parse_vec_component(parts.next(), prefix)?;

    if parts.next().is_some() {
        return Err(ParseReadingError::InvalidFormat(format!(
            "{prefix} expected exactly 3 values"
        )));
    }

    Ok([x, y, z])
}

fn parse_vec_component(value: Option<&str>, field: &str) -> Result<f32, ParseReadingError> {
    let value = value.ok_or_else(|| {
        ParseReadingError::InvalidFormat(format!("{field} expected exactly 3 values"))
    })?;

    value.parse().map_err(|_| ParseReadingError::InvalidNumber {
        field: field.trim_end_matches('=').to_owned(),
        value: value.to_owned(),
    })
}

fn require_prefix<'a>(field: Option<&'a str>, prefix: &str) -> Result<&'a str, ParseReadingError> {
    let field = field.ok_or_else(|| {
        ParseReadingError::InvalidFormat(format!("missing field with prefix {prefix}"))
    })?;

    field.strip_prefix(prefix).ok_or_else(|| {
        ParseReadingError::InvalidFormat(format!(
            "expected field with prefix {prefix}, got {field}"
        ))
    })
}

#[derive(Debug)]
enum ParseReadingError {
    InvalidFormat(String),
    InvalidNumber { field: String, value: String },
}

impl fmt::Display for ParseReadingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidFormat(message) => write!(f, "{message}"),
            Self::InvalidNumber { field, value } => {
                write!(f, "invalid numeric value for {field}: {value}")
            }
        }
    }
}

impl Error for ParseReadingError {}
