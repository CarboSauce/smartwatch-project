// Wygenerowane w notebooku (WEDA-FALL), nie edytuj ręcznie.
#![allow(clippy::collapsible_else_if)]

/// Częstotliwość próbkowania, na której uczono model (LSM6DS3: 52 Hz).
pub const FS: usize = 50;
/// Słabszych wstrząsów nie analizujemy (98% upadków z treningu było mocniejszych).
pub const TRIGGER_G: f32 = 2.91;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Features {
    pub acc_max: f32,
    pub acc_min: f32,
    pub acc_mean: f32,
    pub acc_std: f32,
    pub acc_range: f32,
    pub gyr_max: f32,
    pub gyr_mean: f32,
    pub gyr_std: f32,
    pub post_acc_mean: f32,
    pub post_acc_std: f32,
}

/// true = upadek
pub fn is_fall(f: &Features) -> bool {
    if f.acc_range <= 3.779177 {
        if f.acc_std <= 0.369787 {
            if f.post_acc_std <= 0.004188 {
                if f.gyr_mean <= 34.435504 {
                    true
                } else {
                    false
                }
            } else {
                if f.acc_mean <= 1.079504 {
                    false
                } else {
                    if f.post_acc_mean <= 1.001348 {
                        if f.gyr_mean <= 70.313389 {
                            false
                        } else {
                            true
                        }
                    } else {
                        false
                    }
                }
            }
        } else {
            if f.post_acc_mean <= 1.049533 {
                if f.gyr_mean <= 120.559261 {
                    if f.acc_mean <= 1.051511 {
                        if f.acc_max <= 3.010522 {
                            if f.acc_range <= 2.427440 {
                                false
                            } else {
                                true
                            }
                        } else {
                            false
                        }
                    } else {
                        if f.post_acc_std <= 0.055679 {
                            if f.gyr_std <= 135.259834 {
                                true
                            } else {
                                false
                            }
                        } else {
                            if f.gyr_max <= 321.225861 {
                                false
                            } else {
                                true
                            }
                        }
                    }
                } else {
                    if f.acc_range <= 3.627756 {
                        if f.acc_min <= 0.675425 {
                            false
                        } else {
                            true
                        }
                    } else {
                        if f.post_acc_mean <= 0.995820 {
                            false
                        } else {
                            true
                        }
                    }
                }
            } else {
                if f.acc_mean <= 1.086821 {
                    if f.gyr_max <= 439.652435 {
                        false
                    } else {
                        if f.acc_std <= 0.434693 {
                            false
                        } else {
                            true
                        }
                    }
                } else {
                    if f.acc_min <= 0.222369 {
                        if f.acc_min <= 0.213365 {
                            false
                        } else {
                            true
                        }
                    } else {
                        false
                    }
                }
            }
        }
    } else {
        if f.post_acc_mean <= 1.283737 {
            if f.acc_std <= 0.470722 {
                if f.gyr_std <= 116.506493 {
                    false
                } else {
                    if f.acc_std <= 0.425415 {
                        false
                    } else {
                        true
                    }
                }
            } else {
                if f.gyr_mean <= 199.551857 {
                    if f.acc_std <= 0.595580 {
                        if f.gyr_mean <= 134.000694 {
                            if f.gyr_std <= 141.465446 {
                                true
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    } else {
                        if f.post_acc_std <= 0.026967 {
                            false
                        } else {
                            if f.acc_max <= 3.978264 {
                                false
                            } else {
                                true
                            }
                        }
                    }
                } else {
                    if f.acc_max <= 5.161898 {
                        false
                    } else {
                        if f.post_acc_mean <= 1.191407 {
                            true
                        } else {
                            if f.post_acc_mean <= 1.227145 {
                                false
                            } else {
                                true
                            }
                        }
                    }
                }
            }
        } else {
            if f.gyr_std <= 280.136620 {
                if f.acc_max <= 5.669825 {
                    false
                } else {
                    if f.acc_max <= 5.697237 {
                        true
                    } else {
                        false
                    }
                }
            } else {
                true
            }
        }
    }
}
