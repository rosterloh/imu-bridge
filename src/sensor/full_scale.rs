#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, defmt::Format)]
pub struct AccelFullScale {
    g: u16,
}

impl AccelFullScale {
    pub const fn from_g(g: u16) -> Self {
        Self { g }
    }

    pub const fn g(self) -> u16 {
        self.g
    }

    pub const fn mg(self) -> u32 {
        self.g as u32 * 1000
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, defmt::Format)]
pub struct GyroFullScale {
    thousandths_dps: u32,
}

impl GyroFullScale {
    pub const fn from_dps(dps: u16) -> Self {
        Self {
            thousandths_dps: dps as u32 * 1000,
        }
    }

    pub(crate) const fn from_thousandths_dps(thousandths_dps: u32) -> Self {
        Self { thousandths_dps }
    }

    pub(crate) const fn thousandths_dps(self) -> u32 {
        self.thousandths_dps
    }

    pub fn as_dps(self) -> f32 {
        self.thousandths_dps as f32 / 1000.0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, defmt::Format)]
pub struct FullScaleSelection {
    pub accel: AccelFullScale,
    pub gyro: GyroFullScale,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, defmt::Format)]
pub struct OutputDataRate {
    millihz: u32,
}

impl OutputDataRate {
    pub const fn from_hz(hz: u16) -> Self {
        Self {
            millihz: hz as u32 * 1000,
        }
    }

    pub(crate) const fn from_millihz(millihz: u32) -> Self {
        Self { millihz }
    }

    pub(crate) const fn millihz(self) -> u32 {
        self.millihz
    }

    pub fn as_hz(self) -> f32 {
        self.millihz as f32 / 1000.0
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, defmt::Format)]
pub struct OdrSelection {
    pub accel: OutputDataRate,
    pub gyro: OutputDataRate,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, defmt::Format)]
pub struct SensorSettings {
    pub full_scale: FullScaleSelection,
    pub odr: OdrSelection,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct AccelRangeSetting {
    pub full_scale: AccelFullScale,
    pub register_value: u8,
}

impl AccelRangeSetting {
    pub const fn new(full_scale: AccelFullScale, register_value: u8) -> Self {
        Self {
            full_scale,
            register_value,
        }
    }

    pub fn mg_per_lsb(self) -> f32 {
        self.full_scale.mg() as f32 / 32768.0
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct GyroRangeSetting {
    pub full_scale: GyroFullScale,
    pub register_value: u8,
}

impl GyroRangeSetting {
    pub const fn new(full_scale: GyroFullScale, register_value: u8) -> Self {
        Self {
            full_scale,
            register_value,
        }
    }

    pub fn mdps_per_lsb(self) -> f32 {
        self.full_scale.thousandths_dps() as f32 / 32768.0
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct OdrSetting {
    pub odr: OutputDataRate,
    pub register_value: u8,
}

impl OdrSetting {
    pub const fn new(odr: OutputDataRate, register_value: u8) -> Self {
        Self {
            odr,
            register_value,
        }
    }
}

pub(crate) fn select_accel_range(
    requested: AccelFullScale,
    supported: &[AccelRangeSetting],
) -> AccelRangeSetting {
    select_range_by_key(requested.g() as u32, supported, |entry| {
        entry.full_scale.g() as u32
    })
}

pub(crate) fn select_gyro_range(
    requested: GyroFullScale,
    supported: &[GyroRangeSetting],
) -> GyroRangeSetting {
    select_range_by_key(requested.thousandths_dps(), supported, |entry| {
        entry.full_scale.thousandths_dps()
    })
}

pub(crate) fn select_odr(requested: OutputDataRate, supported: &[OdrSetting]) -> OdrSetting {
    select_range_by_key(requested.millihz(), supported, |entry| entry.odr.millihz())
}

pub(crate) fn log_selected_settings(
    label: &str,
    requested: SensorSettings,
    selected: SensorSettings,
) {
    defmt::info!(
        "{} accel {}g/{}Hz -> {}g/{}Hz gyro {}dps/{}Hz -> {}dps/{}Hz",
        label,
        requested.full_scale.accel.g(),
        requested.odr.accel.as_hz(),
        selected.full_scale.accel.g(),
        selected.odr.accel.as_hz(),
        requested.full_scale.gyro.as_dps(),
        requested.odr.gyro.as_hz(),
        selected.full_scale.gyro.as_dps(),
        selected.odr.gyro.as_hz(),
    );
}

pub(crate) const fn accel_table_is_sorted_and_nonzero(supported: &[AccelRangeSetting]) -> bool {
    if supported.is_empty() {
        return false;
    }

    let mut index = 0;
    while index < supported.len() {
        if supported[index].full_scale.g() == 0 || supported[index].full_scale.mg() == 0 {
            return false;
        }
        if index > 0 && supported[index - 1].full_scale.g() >= supported[index].full_scale.g() {
            return false;
        }
        index += 1;
    }

    true
}

pub(crate) const fn gyro_table_is_sorted_and_nonzero(supported: &[GyroRangeSetting]) -> bool {
    if supported.is_empty() {
        return false;
    }

    let mut index = 0;
    while index < supported.len() {
        if supported[index].full_scale.thousandths_dps() == 0 {
            return false;
        }
        if index > 0
            && supported[index - 1].full_scale.thousandths_dps()
                >= supported[index].full_scale.thousandths_dps()
        {
            return false;
        }
        index += 1;
    }

    true
}

pub(crate) const fn odr_table_is_sorted_and_nonzero(supported: &[OdrSetting]) -> bool {
    if supported.is_empty() {
        return false;
    }

    let mut index = 0;
    while index < supported.len() {
        if supported[index].odr.millihz() == 0 {
            return false;
        }
        if index > 0 && supported[index - 1].odr.millihz() >= supported[index].odr.millihz() {
            return false;
        }
        index += 1;
    }

    true
}

fn select_range_by_key<T: Copy>(requested: u32, supported: &[T], key: impl Fn(T) -> u32) -> T {
    let mut selected = *supported.first().expect("supported ranges cannot be empty");
    let mut selected_distance = requested.abs_diff(key(selected));
    let mut selected_key = key(selected);

    for candidate in supported.iter().copied().skip(1) {
        let candidate_key = key(candidate);
        let candidate_distance = requested.abs_diff(candidate_key);
        if candidate_distance < selected_distance
            || (candidate_distance == selected_distance && candidate_key > selected_key)
        {
            selected = candidate;
            selected_distance = candidate_distance;
            selected_key = candidate_key;
        }
    }

    selected
}

#[cfg(all(test, not(target_os = "none")))]
mod tests {
    use super::{
        AccelFullScale, AccelRangeSetting, GyroFullScale, GyroRangeSetting, OdrSetting,
        OutputDataRate, select_accel_range, select_gyro_range, select_odr,
    };

    #[test]
    fn accel_selection_prefers_exact_match() {
        let supported = [
            AccelRangeSetting::new(AccelFullScale::from_g(2), 0),
            AccelRangeSetting::new(AccelFullScale::from_g(4), 0),
            AccelRangeSetting::new(AccelFullScale::from_g(8), 0),
        ];

        let selected = select_accel_range(AccelFullScale::from_g(4), &supported);

        assert_eq!(selected.full_scale, AccelFullScale::from_g(4));
    }

    #[test]
    fn accel_selection_uses_nearest_supported_value() {
        let supported = [
            AccelRangeSetting::new(AccelFullScale::from_g(2), 0),
            AccelRangeSetting::new(AccelFullScale::from_g(8), 0),
            AccelRangeSetting::new(AccelFullScale::from_g(16), 0),
        ];

        let selected = select_accel_range(AccelFullScale::from_g(5), &supported);

        assert_eq!(selected.full_scale, AccelFullScale::from_g(8));
    }

    #[test]
    fn accel_selection_breaks_ties_upward() {
        let supported = [
            AccelRangeSetting::new(AccelFullScale::from_g(2), 0),
            AccelRangeSetting::new(AccelFullScale::from_g(8), 0),
        ];

        let selected = select_accel_range(AccelFullScale::from_g(5), &supported);

        assert_eq!(selected.full_scale, AccelFullScale::from_g(8));
    }

    #[test]
    fn accel_scale_converts_g_to_mg_per_lsb() {
        let selected = AccelRangeSetting::new(AccelFullScale::from_g(2), 0);

        assert!((selected.mg_per_lsb() - (2000.0 / 32768.0)).abs() < f32::EPSILON);
    }

    #[test]
    fn gyro_selection_prefers_exact_match() {
        let supported = [
            GyroRangeSetting::new(GyroFullScale::from_dps(250), 0),
            GyroRangeSetting::new(GyroFullScale::from_dps(500), 0),
            GyroRangeSetting::new(GyroFullScale::from_dps(1000), 0),
        ];

        let selected = select_gyro_range(GyroFullScale::from_dps(500), &supported);

        assert_eq!(selected.full_scale, GyroFullScale::from_dps(500));
    }

    #[test]
    fn gyro_selection_uses_nearest_supported_value() {
        let supported = [
            GyroRangeSetting::new(GyroFullScale::from_dps(250), 0),
            GyroRangeSetting::new(GyroFullScale::from_dps(500), 0),
            GyroRangeSetting::new(GyroFullScale::from_dps(1000), 0),
        ];

        let selected = select_gyro_range(GyroFullScale::from_dps(700), &supported);

        assert_eq!(selected.full_scale, GyroFullScale::from_dps(500));
    }

    #[test]
    fn gyro_selection_handles_fractional_supported_values() {
        let supported = [
            GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(15_625), 0),
            GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(31_250), 0),
            GyroRangeSetting::new(GyroFullScale::from_thousandths_dps(62_500), 0),
        ];

        let selected = select_gyro_range(GyroFullScale::from_thousandths_dps(20_000), &supported);

        assert_eq!(
            selected.full_scale,
            GyroFullScale::from_thousandths_dps(15_625)
        );
    }

    #[test]
    fn gyro_scale_converts_dps_to_mdps_per_lsb() {
        let selected = GyroRangeSetting::new(GyroFullScale::from_dps(2000), 0);

        assert!((selected.mdps_per_lsb() - (2_000_000.0 / 32768.0)).abs() < f32::EPSILON);
    }

    #[test]
    fn odr_selection_uses_nearest_supported_value() {
        let supported = [
            OdrSetting::new(OutputDataRate::from_millihz(12_500), 0),
            OdrSetting::new(OutputDataRate::from_hz(100), 0),
            OdrSetting::new(OutputDataRate::from_hz(200), 0),
        ];

        let selected = select_odr(OutputDataRate::from_hz(60), &supported);

        assert_eq!(selected.odr, OutputDataRate::from_hz(100));
    }
}
