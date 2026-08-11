function obs = place_obstacles_random(cx, cy, ~, ~, scale, ~, ~)
% place_obstacles_random  Official WRO FE 2026 randomization procedure.
%   Implements the coin-and-cards system from the official
%   fe-randomization-app (World-Robot-Olympiad-Association):
%     1. One random section gets a single sign at X2 (coin → colour)
%     2. The other 3 straight sections each draw a card (1 or 2 signs)
%     3. At least one section must hold a mixed-colour same-row pair
%        (T1+T2 or T3+T4) — forces a lane change within one section
%   Result: always 4-7 signs, never 2 on the same radius, X seats
%   only ever hold single signs.
%
%   Seat geometry (official app coordinates, metres from mat centre):
%     outer row (400 mm from outer wall): T4  X2  T3  at 1.10 m
%     inner row (600 mm from outer wall): T2  X1  T1  at 0.90 m
%     tangential slots: -0.5, 0, +0.5 m

% Seat ids as [row slot]: row 1 = outer (1.10 m), row 2 = inner (0.90 m)
T4=[1 1]; X2=[1 2]; T3=[1 3];
T2=[2 1]; X1=[2 2]; T1=[2 3];

% Official card deck: {seats (n x 2), colours ('g'/'r' per sign)}
deck = { ...
  {T1,'g'}; {T1,'r'}; {X1,'g'}; {X1,'r'}; {T2,'g'}; {T2,'r'}; ...
  {T3,'g'}; {T3,'r'}; {X2,'g'}; {X2,'r'}; {T4,'g'}; {T4,'r'}; ...
  {[T3;T2],'gg'}; {[T3;T2],'gr'}; {[T3;T2],'rg'}; {[T3;T2],'rr'}; ...
  {[T1;T4],'gg'}; {[T1;T4],'gr'}; {[T1;T4],'rg'}; {[T1;T4],'rr'}; ...
  {[T1;T2],'gg'}; {[T1;T2],'gr'}; {[T1;T2],'rg'}; {[T1;T2],'rr'}; ...
  {[T3;T4],'gg'}; {[T3;T4],'gr'}; {[T3;T4],'rg'}; {[T3;T4],'rr'} };
required_cards = [22 23 26 27];   % mixed-colour T1+T2 / T3+T4 pairs

% The robot starts in the South section (zone Z3, driving CCW).
% Official rule: the seats directly in front of the start zone are
% forbidden — for Z3 + CCW these are X1 and X2 of the start section.
START_SEC = 3;

% 1. Coin: pick the section that gets the single X2 sign + its colour.
%    It can never be the start section (X2 there is forbidden).
sec_order = randperm(4);
if sec_order(1) == START_SEC
    tmp = sec_order(1); sec_order(1) = sec_order(2); sec_order(2) = tmp;
end
x2_sec    = sec_order(1);
colours   = 'gr';
x2_colour = colours(randi(2));

% Remove the matching X2 single card from the deck (official rule)
if x2_colour == 'g', deck(9) = []; else, deck(10) = []; end
n_deck = numel(deck);

% 2. Draw one card per remaining section (without replacement)
card_idx = randperm(n_deck, 3);

% 2b. Start-section restrictions: no X seats (in front of the start zone)
%     AND no outer-row seats — the parking lot occupies the outer wall, so
%     signs in this section sit only on the inner row (official rule).
for i = 1:3
    if sec_order(i+1) == START_SEC
        guard = 0;
        while (any(deck{card_idx(i)}{1}(:,2) == 2) || ...
               any(deck{card_idx(i)}{1}(:,1) == 1)) && guard < 100
            card_idx(i) = randi(n_deck);
            guard = guard + 1;
        end
    end
end

% 3. Ensure one required mixed-colour pair is on the field
has_required = false;
for c = card_idx
    seats = deck{c}{1};  cols = deck{c}{2};
    if size(seats,1) == 2 && cols(1) ~= cols(2) && seats(1,1) == seats(2,1)
        has_required = true;   % same row (T1+T2 or T3+T4), mixed colours
    end
end
if ~has_required
    % Overwrite one drawn card with a required mixed pair — but never on
    % the start section (T3+T4 is outer-row, illegal beside the parking lot)
    req = { {[T1;T2],'gr'}, {[T1;T2],'rg'}, {[T3;T4],'gr'}, {[T3;T4],'rg'} };
    pos_ok = find(sec_order(2:4) ~= START_SEC);
    card_idx_pos = pos_ok(randi(numel(pos_ok)));
    forced = req{randi(4)};
    deck{card_idx(card_idx_pos)} = forced;
end

% ── Convert sections + cards into mat coordinates ─────────────────────────
r_row  = [1.10, 0.90] / scale;    % row 1 = outer, row 2 = inner
t_slot = [-0.5, 0, 0.5] / scale;

obs = struct('color',{},'x',{},'y',{});
    function add_sign(section, seat, colour_char)
        r = r_row(seat(1));
        t = t_slot(seat(2));
        switch section
            case 1, x = cx + t; y = cy - r;   % North
            case 2, x = cx + r; y = cy + t;   % East
            case 3, x = cx + t; y = cy + r;   % South
            case 4, x = cx - r; y = cy + t;   % West
        end
        k = numel(obs) + 1;
        if colour_char == 'r', obs(k).color = 'red';
        else,                  obs(k).color = 'green'; end
        obs(k).x = x;
        obs(k).y = y;
    end

% X2 single in the coin section
add_sign(x2_sec, X2, x2_colour);

% Card sections
for i = 1:3
    card  = deck{card_idx(i)};
    seats = card{1};
    cols  = card{2};
    for s = 1:size(seats,1)
        add_sign(sec_order(i+1), seats(s,:), cols(s));
    end
end

n_red   = sum(strcmp({obs.color},'red'));
n_green = sum(strcmp({obs.color},'green'));
fprintf('Official randomization: %d signs (%d red, %d green), X2 single in section %d\n', ...
        numel(obs), n_red, n_green, x2_sec);
end
