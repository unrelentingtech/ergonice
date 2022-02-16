use keyberon::{
    action::{d, k, l, Action::*, HoldTapConfig, m},
    key_code::KeyCode::*,
};

type Action = keyberon::action::Action<()>;

const TIMEOUT_TICKS: u16 = 250 / 4;

const CTRL_ESC: Action = HoldTap {
    timeout: TIMEOUT_TICKS,
    tap_hold_interval: 0,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    hold: &k(LCtrl),
    tap: &k(Escape),
};

const LSHIFT_P: Action = HoldTap {
    timeout: TIMEOUT_TICKS,
    tap_hold_interval: 0,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    hold: &k(LShift),
    tap: &m(&[LShift, Kb9]),
};

const RSHIFT_P: Action = HoldTap {
    timeout: TIMEOUT_TICKS,
    tap_hold_interval: 0,
    config: HoldTapConfig::HoldOnOtherKeyPress,
    hold: &k(RShift),
    tap: &m(&[RShift, Kb0]),
};

// these were intended as "macro" keys but I don't have that much use tbh
const M1: Action = k(Copy);
const M2: Action = k(Paste);
const M3: Action = k(Undo);
const M4: Action = k(MediaCoffee);

// last column is things other than matrix keys: knob rotation and click, external button connectors

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<()> = &[
    &[ // 0: default
        &[NoOp,        k(Kb1),        k(Kb2),      k(Kb3),  k(Kb4),  k(Kb5),   NoOp,     NoOp,       k(Kb6),    k(Kb7),    k(Kb8),          k(Kb9),      k(Kb0),     k(Minus),     k(Equal),      k(VolDown)],
        &[k(Grave),    k(Q),          k(W),        k(E),    k(R),    k(T),     NoOp,     NoOp,       k(Y),      k(U),      k(I),            k(O),        k(P),       k(LBracket),  k(RBracket),   k(VolUp)],
        &[CTRL_ESC,    k(A),          k(S),        k(D),    k(F),    k(G),     M2,       M3,         k(H),      k(J),      k(K),            k(L),        k(SColon),  k(Quote),     k(Bslash),     k(MediaPlayPause)],
        &[LSHIFT_P,    k(Z),          k(X),        k(C),    k(V),    k(B),     M1,       M4,         k(N),      k(M),      k(Comma),        k(Dot),      RSHIFT_P,   k(Up),        k(Slash),      k(LCtrl)],
        &[l(1),        k(ScrollLock), k(Menu),     k(LAlt), k(LGui), k(Space), k(Tab),   k(BSpace),  k(Enter),  k(Delete), k(Application),  NoOp,        k(Left),    k(Down),      k(Right),      k(Enter)],
    ],
    &[ // 1: fn
        &[Trans,       k(F1),         k(F2),       k(F3),   k(F4),   k(F5),    Trans,    Trans,      k(F6),     k(F7),     k(F8),           k(F9),       k(F10),     k(F11),       k(F12),        k(MediaScrollDown)],
        &[Trans,       k(F13),        k(F14),      k(F15),  k(F16),  k(F17),   Trans,    Trans,      k(F18),    k(F19),    k(F20),          k(F21),      k(F22),     k(F23),       k(F24),        k(MediaScrollUp)],
        &[k(Escape),   Trans,         Trans,       Trans,   Trans,   Trans,    d(2),     NoOp,       Trans,     Trans,     Trans,           Trans,       Trans,      Trans,        Trans,         k(Mute)],
        &[k(CapsLock), Trans,         Trans,       Trans,   Trans,   Trans,    d(0),     d(3),       Trans,     Trans,     Trans,           Trans,       Trans,      k(PgUp),      Trans,         Trans],
        &[Trans,       k(SysReq),     k(PScreen),  k(RAlt), Trans,   Trans,    Trans,    Trans,      Trans,     k(Insert), k(Pause),        Trans,       k(Home),    k(PgDown),    k(End),        Trans],
    ],
    &[ // 2: numpad (the last few columns are ortholinear so why not)
        &[NoOp,        k(Kb1),        k(Kb2),      k(Kb3),  k(Kb4),  k(Kb5),   NoOp,     NoOp,       k(Kb6),    k(Kb7),    k(Kb8),          k(KpMinus),  k(NumLock), k(KpSlash),   k(KpAsterisk), k(VolDown)],
        &[k(Grave),    k(Q),          k(W),        k(E),    k(R),    k(T),     NoOp,     NoOp,       k(Y),      k(U),      k(I),            k(KpPlus),   k(Kp7),     k(Kp8),       k(Kp9),        k(VolUp)],
        &[CTRL_ESC,    k(A),          k(S),        k(D),    k(F),    k(G),     M2,       M3,         k(H),      k(J),      k(K),            k(L),        k(Kp4),     k(Kp5),       k(Kp6),        k(MediaPlayPause)],
        &[LSHIFT_P,    k(Z),          k(X),        k(C),    k(V),    k(B),     M1,       M4,         k(N),      k(M),      k(Comma),        k(Dot),      k(Kp1),     k(Kp2),       k(Kp3),        k(LCtrl)],
        &[l(1),        k(ScrollLock), k(Menu),     k(LAlt), k(LGui), k(Space), k(Tab),   k(BSpace),  k(Enter),  k(Delete), k(Application),  NoOp,        k(Kp0),     k(KpEnter),   k(KpDot),      k(Enter)],
    ],
    &[ // 3: firmware colemak (useful for unusal places like firmware settings)
        &[NoOp,        k(Kb1),        k(Kb2),      k(Kb3),  k(Kb4),  k(Kb5),   NoOp,     NoOp,       k(Kb6),    k(Kb7),    k(Kb8),          k(Kb9),      k(Kb0),     k(Minus),     k(Equal),      k(VolDown)],
        &[k(Grave),    k(Q),          k(W),        k(F),    k(P),    k(G),     NoOp,     NoOp,       k(J),      k(L),      k(U),            k(Y),        k(SColon),  k(LBracket),  k(RBracket),   k(VolUp)],
        &[CTRL_ESC,    k(A),          k(R),        k(S),    k(T),    k(D),     M2,       M3,         k(H),      k(N),      k(E),            k(I),        k(O),       k(Quote),     k(Bslash),     k(MediaPlayPause)],
        &[LSHIFT_P,    k(Z),          k(X),        k(C),    k(V),    k(B),     M1,       M4,         k(K),      k(M),      k(Comma),        k(Dot),      RSHIFT_P,   k(Up),        k(Slash),      k(LCtrl)],
        &[l(1),        k(ScrollLock), k(Menu),     k(LAlt), k(LGui), k(Space), k(Tab),   k(BSpace),  k(Enter),  k(Delete), k(Application),  NoOp,        k(Left),    k(Down),      k(Right),      k(Enter)],
    ],
    // &[
        // &[Trans,       Trans,         Trans,       Trans,   Trans,   Trans,    Trans,    Trans,      Trans,     Trans,     Trans,           Trans,       Trans,      Trans,        Trans,         Trans],
        // &[Trans,       Trans,         Trans,       Trans,   Trans,   Trans,    Trans,    Trans,      Trans,     Trans,     Trans,           Trans,       Trans,      Trans,        Trans,         Trans],
        // &[Trans,       Trans,         Trans,       Trans,   Trans,   Trans,    Trans,    Trans,      Trans,     Trans,     Trans,           Trans,       Trans,      Trans,        Trans,         Trans],
        // &[Trans,       Trans,         Trans,       Trans,   Trans,   Trans,    Trans,    Trans,      Trans,     Trans,     Trans,           Trans,       Trans,      Trans,        Trans,         Trans],
        // &[Trans,       Trans,         Trans,       Trans,   Trans,   Trans,    Trans,    Trans,      Trans,     Trans,     Trans,           Trans,       Trans,      Trans,        Trans,         Trans],
    // ],
];
